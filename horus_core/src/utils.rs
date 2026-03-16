//! Internal utility functions for HORUS core
//!
//! Shared helpers used across multiple modules.

// ============================================================================
// Branch Prediction Hints (HOT PATH OPTIMIZATION)
// ============================================================================

/// Hint that a branch is unlikely to be taken (cold path)
#[inline(always)]
#[cold]
fn cold() {}

/// Mark a condition as unlikely (branch prediction hint for cold paths).
/// Use for: error paths, rare conditions, buffer-full checks.
#[inline(always)]
pub(crate) fn unlikely(b: bool) -> bool {
    if b {
        cold()
    }
    b
}

// ============================================================================
// Fuzzy String Matching (for "did you mean?" suggestions)
// ============================================================================

/// Compute Levenshtein edit distance between two strings.
///
/// Returns the minimum number of single-character edits (insert, delete, substitute)
/// needed to transform `a` into `b`. No external dependencies — pure DP.
pub(crate) fn levenshtein_distance(a: &str, b: &str) -> usize {
    let a_len = a.len();
    let b_len = b.len();

    if a_len == 0 {
        return b_len;
    }
    if b_len == 0 {
        return a_len;
    }

    // Single-row DP: prev[j] = distance(a[..i], b[..j])
    let mut prev: Vec<usize> = (0..=b_len).collect();
    let mut curr = vec![0; b_len + 1];

    for (i, a_ch) in a.chars().enumerate() {
        curr[0] = i + 1;
        for (j, b_ch) in b.chars().enumerate() {
            let cost = if a_ch == b_ch { 0 } else { 1 };
            curr[j + 1] = (prev[j] + cost)
                .min(prev[j + 1] + 1) // deletion
                .min(curr[j] + 1); // insertion
        }
        std::mem::swap(&mut prev, &mut curr);
    }

    prev[b_len]
}

/// Find the closest match to `query` among `candidates` within `max_distance`.
///
/// Returns the best match (lowest distance, > 0) or `None` if no candidate
/// is close enough. Used for "did you mean?" suggestions in error messages.
pub(crate) fn suggest_similar<'a>(
    query: &str,
    candidates: impl IntoIterator<Item = &'a str>,
    max_distance: usize,
) -> Option<String> {
    candidates
        .into_iter()
        .filter_map(|c| {
            let d = levenshtein_distance(query, c);
            if d > 0 && d <= max_distance {
                Some((c, d))
            } else {
                None
            }
        })
        .min_by_key(|(_, d)| *d)
        .map(|(name, _)| name.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn levenshtein_identical() {
        assert_eq!(levenshtein_distance("hello", "hello"), 0);
    }

    #[test]
    fn levenshtein_empty() {
        assert_eq!(levenshtein_distance("", "abc"), 3);
        assert_eq!(levenshtein_distance("abc", ""), 3);
        assert_eq!(levenshtein_distance("", ""), 0);
    }

    #[test]
    fn levenshtein_single_char() {
        assert_eq!(levenshtein_distance("a", "b"), 1);
        assert_eq!(levenshtein_distance("abc", "adc"), 1);
    }

    #[test]
    fn levenshtein_insert_delete() {
        assert_eq!(levenshtein_distance("cmd_vel", "cmd_velocity"), 5);
        assert_eq!(levenshtein_distance("base_lnk", "base_link"), 1);
    }

    #[test]
    fn suggest_finds_closest() {
        let candidates = ["cmd_vel", "odom", "imu", "laser_scan"];
        assert_eq!(
            suggest_similar("cmd_vl", candidates.iter().copied(), 3),
            Some("cmd_vel".into())
        );
    }

    #[test]
    fn suggest_none_when_too_far() {
        let candidates = ["cmd_vel", "odom"];
        assert_eq!(
            suggest_similar("completely_different", candidates.iter().copied(), 3),
            None
        );
    }

    #[test]
    fn suggest_none_for_exact_match() {
        let candidates = ["cmd_vel"];
        assert_eq!(
            suggest_similar("cmd_vel", candidates.iter().copied(), 3),
            None,
            "exact match should return None (distance 0)"
        );
    }

    #[test]
    fn suggest_base_link_typo() {
        let candidates = ["base_link", "odom", "world"];
        assert_eq!(
            suggest_similar("base_lnk", candidates.iter().copied(), 3),
            Some("base_link".into())
        );
    }
}
