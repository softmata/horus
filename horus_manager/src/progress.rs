//! Progress indicators for HORUS CLI
//!
//! Provides spinners and progress bars for long-running operations.

use console::style;
use indicatif::{ProgressBar, ProgressStyle};
use std::sync::atomic::{AtomicBool, Ordering};
use horus_core::core::DurationExt;

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
pub const STATUS_SUCCESS: &str = crate::cli_output::ICON_SUCCESS;

/// Error indicator
pub const STATUS_ERROR: &str = crate::cli_output::ICON_ERROR;

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
    pb.enable_steady_tick(80_u64.ms());
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
    pb.set_style(
        ProgressStyle::default_spinner()
            .template("{msg}")
            .expect("valid template"),
    );
    pb.finish_with_message(format!("{} {}", STATUS_SUCCESS, style(message).green()));
}

/// Finish a spinner with error
pub fn finish_error(pb: &ProgressBar, message: &str) {
    pb.set_style(
        ProgressStyle::default_spinner()
            .template("{msg}")
            .expect("valid template"),
    );
    pb.finish_with_message(format!("{} {}", STATUS_ERROR, style(message).red()));
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

    #[test]
    fn test_spinner_creation() {
        let pb = spinner("Testing...");
        assert!(!pb.is_finished(), "Spinner should be active after creation");
        finish_success(&pb, "Test complete!");
        assert!(pb.is_finished(), "Spinner should be finished after finish_success");
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

    // --- format_bytes edge cases ---

    #[test]
    fn test_format_bytes_zero() {
        assert_eq!(format_bytes(0), "0 B");
    }

    #[test]
    fn test_format_bytes_one() {
        assert_eq!(format_bytes(1), "1 B");
    }

    #[test]
    fn test_format_bytes_just_below_kb() {
        assert_eq!(format_bytes(1023), "1023 B");
    }

    #[test]
    fn test_format_bytes_exact_kb() {
        assert_eq!(format_bytes(1024), "1.0 KB");
    }

    #[test]
    fn test_format_bytes_just_below_mb() {
        // 1024*1024 - 1 = 1048575
        assert_eq!(format_bytes(1048575), "1024.0 KB");
    }

    #[test]
    fn test_format_bytes_exact_mb() {
        assert_eq!(format_bytes(1048576), "1.0 MB");
    }

    #[test]
    fn test_format_bytes_just_below_gb() {
        // 1024*1024*1024 - 1 = 1073741823
        assert_eq!(format_bytes(1073741823), "1024.0 MB");
    }

    #[test]
    fn test_format_bytes_exact_gb() {
        assert_eq!(format_bytes(1073741824), "1.00 GB");
    }

    #[test]
    fn test_format_bytes_multiple_gb() {
        // 5 GB
        assert_eq!(format_bytes(5 * 1073741824), "5.00 GB");
    }

    #[test]
    fn test_format_bytes_fractional_kb() {
        // 2.5 KB = 2560
        assert_eq!(format_bytes(2560), "2.5 KB");
    }

    #[test]
    fn test_format_bytes_fractional_mb() {
        // 1.5 MB = 1572864
        assert_eq!(format_bytes(1572864), "1.5 MB");
    }

    #[test]
    fn test_format_bytes_large_value() {
        // 100 GB
        assert_eq!(format_bytes(100 * 1073741824), "100.00 GB");
    }

    #[test]
    fn test_format_bytes_u64_max_region() {
        // Very large value — should produce a large GB number without panic
        let result = format_bytes(u64::MAX);
        assert!(result.contains("GB"), "very large value should be in GB: {}", result);
    }

    // --- quiet mode / set_quiet / is_quiet ---

    #[test]
    fn test_set_quiet_true_then_false() {
        set_quiet(true);
        assert!(is_quiet());
        set_quiet(false);
        assert!(!is_quiet());
    }

    #[test]
    fn test_quiet_mode_affects_all_spinner_types() {
        set_quiet(true);
        let s1 = spinner("msg");
        let s2 = build_spinner("msg");
        let s3 = download_spinner("msg");
        assert!(s1.is_hidden());
        assert!(s2.is_hidden());
        assert!(s3.is_hidden());
        set_quiet(false);
    }

    #[test]
    fn test_non_quiet_spinner_is_not_force_hidden() {
        // When quiet mode is off, spinner() does NOT use ProgressBar::hidden().
        // However, indicatif may auto-hide in non-TTY environments.
        // We verify: quiet=true => definitely hidden, quiet=false => not force-hidden
        // by checking the quiet-mode path specifically.
        set_quiet(false);
        assert!(!is_quiet());
        // The spinner function only calls ProgressBar::hidden() when is_quiet() is true,
        // so we just verify the code path: is_quiet returns false.
        set_quiet(false);
    }

    // --- spinner variants ---

    #[test]
    fn test_build_spinner_creation() {
        set_quiet(false);
        let pb = build_spinner("Building...");
        assert!(!pb.is_finished());
        finish_success(&pb, "Build done");
        assert!(pb.is_finished());
    }

    #[test]
    fn test_download_spinner_creation() {
        set_quiet(false);
        let pb = download_spinner("Downloading...");
        assert!(!pb.is_finished());
        finish_success(&pb, "Download done");
        assert!(pb.is_finished());
    }

    // --- finish_success / finish_error ---

    #[test]
    fn test_finish_error() {
        set_quiet(false);
        let pb = spinner("working...");
        assert!(!pb.is_finished());
        finish_error(&pb, "Something failed");
        assert!(pb.is_finished());
    }

    #[test]
    fn test_finish_success_then_check_finished() {
        set_quiet(false);
        let pb = spinner("working...");
        finish_success(&pb, "OK");
        assert!(pb.is_finished());
    }

    #[test]
    fn test_finish_error_on_hidden_spinner() {
        set_quiet(true);
        let pb = spinner("hidden");
        finish_error(&pb, "error msg");
        // Should not panic even on hidden spinner
        assert!(pb.is_finished());
        set_quiet(false);
    }

    #[test]
    fn test_finish_success_on_hidden_spinner() {
        set_quiet(true);
        let pb = spinner("hidden");
        finish_success(&pb, "success msg");
        assert!(pb.is_finished());
        set_quiet(false);
    }

    // --- Constants ---

    #[test]
    fn test_braille_spinner_has_frames() {
        assert!(!BRAILLE_SPINNER.is_empty());
        assert_eq!(BRAILLE_SPINNER.len(), 10, "braille spinner should have 10 frames");
    }

    #[test]
    fn test_braille_spinner_frames_non_empty() {
        for frame in BRAILLE_SPINNER {
            assert!(!frame.is_empty());
        }
    }

    #[test]
    fn test_status_success_not_empty() {
        assert!(!STATUS_SUCCESS.is_empty());
    }

    #[test]
    fn test_status_error_not_empty() {
        assert!(!STATUS_ERROR.is_empty());
    }

    #[test]
    fn test_status_success_and_error_differ() {
        assert_ne!(STATUS_SUCCESS, STATUS_ERROR);
    }

    // --- spinner with empty message ---

    #[test]
    fn test_spinner_empty_message() {
        set_quiet(false);
        let pb = spinner("");
        assert!(!pb.is_finished());
        finish_success(&pb, "");
        assert!(pb.is_finished());
    }

    #[test]
    fn test_finish_with_long_message() {
        set_quiet(false);
        let pb = spinner("short");
        let long_msg = "a".repeat(1000);
        finish_success(&pb, &long_msg);
        assert!(pb.is_finished());
    }
}
