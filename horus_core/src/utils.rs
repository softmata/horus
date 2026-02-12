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

/// Mark a condition as likely (branch prediction hint for hot paths).
/// Use for: success paths, common conditions.
#[inline(always)]
pub(crate) fn likely(b: bool) -> bool {
    if !b {
        cold()
    }
    b
}
