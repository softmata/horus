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

#[cfg(test)]
mod tests {
    use super::*;

    // ── Constants ───────────────────────────────────────────────────────

    #[test]
    fn icon_success_is_checkmark() {
        assert_eq!(ICON_SUCCESS, "\u{2713}");
        assert_eq!(ICON_SUCCESS, "✓");
    }

    #[test]
    fn icon_error_is_ballot_x() {
        assert_eq!(ICON_ERROR, "\u{2717}");
        assert_eq!(ICON_ERROR, "✗");
    }

    #[test]
    fn icon_warn_is_warning_sign() {
        assert_eq!(ICON_WARN, "\u{26a0}");
        assert_eq!(ICON_WARN, "⚠");
    }

    #[test]
    fn icon_info_is_right_pointing_triangle() {
        assert_eq!(ICON_INFO, "\u{25b6}");
        assert_eq!(ICON_INFO, "▶");
    }

    #[test]
    fn icon_hint_is_middle_dot() {
        assert_eq!(ICON_HINT, "\u{00b7}");
        assert_eq!(ICON_HINT, "·");
    }

    #[test]
    fn all_icons_are_single_char() {
        // Each icon should be exactly one Unicode character
        assert_eq!(ICON_SUCCESS.chars().count(), 1);
        assert_eq!(ICON_ERROR.chars().count(), 1);
        assert_eq!(ICON_WARN.chars().count(), 1);
        assert_eq!(ICON_INFO.chars().count(), 1);
        assert_eq!(ICON_HINT.chars().count(), 1);
    }

    // ── safe_truncate: basic behavior ──────────────────────────────────

    #[test]
    fn truncate_short_string_unchanged() {
        assert_eq!(safe_truncate("hello", 10), "hello");
    }

    #[test]
    fn truncate_exact_length_unchanged() {
        assert_eq!(safe_truncate("hello", 5), "hello");
    }

    #[test]
    fn truncate_longer_string() {
        let result = safe_truncate("hello world", 8);
        assert!(result.ends_with("..."));
        assert!(result.len() <= 8);
        assert_eq!(result, "hello...");
    }

    #[test]
    fn truncate_empty_string() {
        assert_eq!(safe_truncate("", 0), "");
        assert_eq!(safe_truncate("", 5), "");
        assert_eq!(safe_truncate("", 100), "");
    }

    #[test]
    fn truncate_single_char_within_limit() {
        assert_eq!(safe_truncate("a", 1), "a");
        assert_eq!(safe_truncate("a", 5), "a");
    }

    // ── safe_truncate: ellipsis mechanics ──────────────────────────────

    #[test]
    fn truncate_at_exactly_ellipsis_len() {
        // max_len == 3 means all bytes go to "..."
        let result = safe_truncate("abcdef", 3);
        assert_eq!(result, "...");
    }

    #[test]
    fn truncate_leaves_room_for_ellipsis() {
        // max_len == 4 means 1 char + "..."
        let result = safe_truncate("abcdef", 4);
        assert_eq!(result, "a...");
    }

    #[test]
    fn truncate_max_len_less_than_ellipsis() {
        // max_len == 2 is less than "..." (3 bytes).
        // saturating_sub gives target=0, so we get just "..."
        let result = safe_truncate("abcdef", 2);
        assert_eq!(result, "...");
    }

    #[test]
    fn truncate_max_len_one() {
        let result = safe_truncate("abcdef", 1);
        assert_eq!(result, "...");
    }

    #[test]
    fn truncate_max_len_zero() {
        let result = safe_truncate("abcdef", 0);
        assert_eq!(result, "...");
    }

    // ── safe_truncate: UTF-8 multibyte safety ──────────────────────────

    #[test]
    fn truncate_respects_utf8_boundary_2byte() {
        // 'é' is 2 bytes (U+00E9). "café" = 5 bytes (c=1, a=1, f=1, é=2)
        let s = "café";
        assert_eq!(s.len(), 5);
        // max_len=5 => no truncation
        assert_eq!(safe_truncate(s, 5), "café");
        // max_len=4 => needs truncation, target=1, but can't split 'é'
        let r = safe_truncate(s, 4);
        assert!(r.ends_with("..."));
        // Must not produce invalid UTF-8 — the fact this compiles and
        // we can call .len() on it proves it's valid UTF-8
        assert!(r.len() <= 7); // "..." is 3 bytes itself, but total should be reasonable
    }

    #[test]
    fn truncate_respects_utf8_boundary_3byte() {
        // '€' is 3 bytes (U+20AC). "a€b" = 5 bytes
        let s = "a€b";
        assert_eq!(s.len(), 5);
        let r = safe_truncate(s, 4);
        assert!(r.ends_with("..."));
        // target = 4-3 = 1, which is the 'a' boundary
        assert_eq!(r, "a...");
    }

    #[test]
    fn truncate_respects_utf8_boundary_4byte() {
        // '𝄞' is 4 bytes (U+1D11E). "x𝄞y" = 6 bytes
        let s = "x𝄞y";
        assert_eq!(s.len(), 6);
        // max_len=5 => target=2, byte 2 is middle of 𝄞, walk back to 1 => "x"
        let r = safe_truncate(s, 5);
        assert_eq!(r, "x...");
    }

    #[test]
    fn truncate_all_multibyte_chars() {
        // "ñoño" = 6 bytes (ñ=2,o=1,ñ=2,o=1)
        let s = "ñoño";
        assert_eq!(s.len(), 6);
        let r = safe_truncate(s, 6);
        assert_eq!(r, "ñoño"); // exact fit
        let r = safe_truncate(s, 5);
        assert!(r.ends_with("..."));
    }

    #[test]
    fn truncate_emoji() {
        // '😀' is 4 bytes
        let s = "😀😀😀";
        assert_eq!(s.len(), 12);
        let r = safe_truncate(s, 10);
        assert!(r.ends_with("..."));
        // target=7, byte 7 is middle of 2nd emoji, walk back to 4 => first emoji
        assert_eq!(r, "😀...");
    }

    #[test]
    fn truncate_mixed_ascii_and_multibyte() {
        let s = "hello wörld";
        // h=1,e=1,l=1,l=1,o=1, =1,w=1,ö=2,r=1,l=1,d=1 = 12 bytes
        assert_eq!(s.len(), 12);
        let r = safe_truncate(s, 10);
        assert!(r.ends_with("..."));
        // target=7, byte 7 is start of 'ö' (2 bytes), so we take up to byte 7
        assert_eq!(r, "hello w...");
    }

    // ── safe_truncate: boundary precision ──────────────────────────────

    #[test]
    fn truncate_returns_valid_utf8_for_all_cut_points() {
        let s = "aé€𝄞"; // 1+2+3+4 = 10 bytes
        assert_eq!(s.len(), 10);
        for max_len in 0..=15 {
            let r = safe_truncate(s, max_len);
            // Must always be valid UTF-8 (this is enforced by being a String)
            // and if truncated, must end with "..."
            if max_len < s.len() {
                assert!(r.ends_with("..."), "max_len={max_len}, result={r:?}");
            }
        }
    }

    #[test]
    fn truncate_long_string() {
        let s = "a".repeat(10_000);
        let r = safe_truncate(&s, 100);
        assert_eq!(r.len(), 100);
        assert!(r.ends_with("..."));
        assert_eq!(&r[..97], &"a".repeat(97));
    }

    // ── Output functions: no-panic smoke tests ─────────────────────────
    //
    // These functions write to stdout/stderr with ANSI colors. We verify
    // they don't panic and that quiet-mode gating is exercised both ways.

    /// Helper to ensure quiet mode is reset after each test.
    fn with_quiet<F: FnOnce()>(quiet: bool, f: F) {
        crate::progress::set_quiet(quiet);
        f();
        crate::progress::set_quiet(false);
    }

    #[test]
    fn success_does_not_panic() {
        with_quiet(false, || success("test success message"));
    }

    #[test]
    fn success_quiet_does_not_panic() {
        with_quiet(true, || success("should be suppressed"));
    }

    #[test]
    fn error_does_not_panic() {
        error("test error message");
    }

    #[test]
    fn error_prints_even_when_quiet() {
        // error should always print — just verify no panic in quiet mode
        with_quiet(true, || error("error in quiet mode"));
    }

    #[test]
    fn warn_does_not_panic() {
        warn("test warning message");
    }

    #[test]
    fn warn_prints_even_when_quiet() {
        with_quiet(true, || warn("warning in quiet mode"));
    }

    #[test]
    fn info_does_not_panic() {
        with_quiet(false, || info("test info message"));
    }

    #[test]
    fn info_quiet_does_not_panic() {
        with_quiet(true, || info("should be suppressed"));
    }

    #[test]
    fn hint_does_not_panic() {
        with_quiet(false, || hint("test hint message"));
    }

    #[test]
    fn hint_quiet_does_not_panic() {
        with_quiet(true, || hint("should be suppressed"));
    }

    #[test]
    fn header_does_not_panic() {
        with_quiet(false, || header("Test Header"));
    }

    #[test]
    fn header_quiet_does_not_panic() {
        with_quiet(true, || header("should be suppressed"));
    }

    #[test]
    fn empty_without_tip_does_not_panic() {
        empty("No items found", None);
    }

    #[test]
    fn empty_with_tip_does_not_panic() {
        with_quiet(false, || empty("No items found", Some("Try running init")));
    }

    #[test]
    fn empty_with_tip_quiet_suppresses_hint() {
        // The main yellow message always prints, but the hint should be suppressed
        with_quiet(true, || empty("No items found", Some("tip suppressed")));
    }

    // ── Output functions: with special content ─────────────────────────

    #[test]
    fn output_functions_handle_empty_string() {
        with_quiet(false, || {
            success("");
            error("");
            warn("");
            info("");
            hint("");
            header("");
            empty("", None);
            empty("", Some(""));
        });
    }

    #[test]
    fn output_functions_handle_unicode() {
        let msg = "日本語テスト — Ñoño — 🤖🦀";
        with_quiet(false, || {
            success(msg);
            error(msg);
            warn(msg);
            info(msg);
            hint(msg);
            header(msg);
            empty(msg, Some(msg));
        });
    }

    #[test]
    fn output_functions_handle_newlines() {
        let msg = "line1\nline2\nline3";
        with_quiet(false, || {
            success(msg);
            error(msg);
            warn(msg);
            info(msg);
        });
    }

    #[test]
    fn output_functions_handle_ansi_in_input() {
        // Input that already contains ANSI codes shouldn't break anything
        let msg = "\x1b[31mred text\x1b[0m";
        with_quiet(false, || {
            success(msg);
            error(msg);
            warn(msg);
        });
    }

    #[test]
    fn output_functions_handle_long_string() {
        let msg = "x".repeat(10_000);
        with_quiet(false, || {
            success(&msg);
            error(&msg);
            warn(&msg);
        });
    }
}
