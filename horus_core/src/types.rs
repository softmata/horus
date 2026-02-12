//! Core types for HORUS messages
//!
//! This module provides internal types used by the message! macro
//! for zero-copy serialization.

use serde::{Deserialize, Serialize};
use std::fmt;

/// Fixed-size string for zero-copy messages.
///
/// This type is used internally by the `message!` macro when users
/// declare `String` fields with `#[max_len = N]` attributes.
///
/// Users should not use this type directly - just use `String` in
/// message definitions and the macro handles the conversion.
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(C)]
pub struct FixedString<const N: usize> {
    data: [u8; N],
    len: u8,
}

impl<const N: usize> FixedString<N> {
    /// Maximum capacity of this fixed string
    pub const CAPACITY: usize = N;

    // Compile-time check: len is u8, so N must not exceed 255
    const _ASSERT_N_FITS_U8: () = assert!(N <= 255, "FixedString<N>: N must be <= 255 (len is u8)");

    /// Create a new empty fixed string
    #[inline]
    pub const fn new() -> Self {
        // Trigger the compile-time assertion
        let _ = Self::_ASSERT_N_FITS_U8;
        Self {
            data: [0; N],
            len: 0,
        }
    }

    /// Create a fixed string from a string slice
    ///
    /// If the string is longer than capacity, it will be truncated.
    #[inline]
    #[allow(clippy::should_implement_trait)]
    pub fn from_str(s: &str) -> Self {
        let mut result = Self::new();
        // Truncate at a valid UTF-8 char boundary to avoid splitting multi-byte characters
        let truncated = if s.len() <= N {
            s
        } else {
            let mut end = N;
            while end > 0 && !s.is_char_boundary(end) {
                end -= 1;
            }
            &s[..end]
        };
        let bytes = truncated.as_bytes();
        result.data[..bytes.len()].copy_from_slice(bytes);
        result.len = bytes.len() as u8;
        result
    }

    /// Get the string contents as a string slice
    #[inline]
    pub fn as_str(&self) -> &str {
        let len = (self.len as usize).min(N);
        // Use checked conversion to handle data from shared memory or bytemuck casts
        // that may contain invalid UTF-8
        std::str::from_utf8(&self.data[..len]).unwrap_or("")
    }

    /// Get the current length
    #[inline]
    pub fn len(&self) -> usize {
        self.len as usize
    }

    /// Check if the string is empty
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Clear the string
    #[inline]
    pub fn clear(&mut self) {
        self.len = 0;
    }
}

impl<const N: usize> Default for FixedString<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> fmt::Debug for FixedString<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self.as_str())
    }
}

impl<const N: usize> fmt::Display for FixedString<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.as_str())
    }
}

impl<const N: usize> From<&str> for FixedString<N> {
    fn from(s: &str) -> Self {
        Self::from_str(s)
    }
}

impl<const N: usize> AsRef<str> for FixedString<N> {
    fn as_ref(&self) -> &str {
        self.as_str()
    }
}

// SAFETY: FixedString<N> is a repr(C) struct with only POD fields
unsafe impl<const N: usize> bytemuck::Pod for FixedString<N> where [u8; N]: bytemuck::Pod {}
unsafe impl<const N: usize> bytemuck::Zeroable for FixedString<N> {}

// Serde support for compatibility
impl<const N: usize> Serialize for FixedString<N> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        serializer.serialize_str(self.as_str())
    }
}

impl<'de, const N: usize> Deserialize<'de> for FixedString<N> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let s = String::deserialize(deserializer)?;
        Ok(Self::from_str(&s))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fixed_string_basic() {
        let s = FixedString::<16>::from_str("hello");
        assert_eq!(s.as_str(), "hello");
        assert_eq!(s.len(), 5);
        assert!(!s.is_empty());
    }

    #[test]
    fn test_fixed_string_truncation() {
        let s = FixedString::<8>::from_str("this is a very long string");
        assert_eq!(s.as_str(), "this is ");
        assert_eq!(s.len(), 8);
    }

    #[test]
    fn test_fixed_string_empty() {
        let s = FixedString::<16>::new();
        assert_eq!(s.as_str(), "");
        assert_eq!(s.len(), 0);
        assert!(s.is_empty());
    }

    #[test]
    fn test_fixed_string_pod() {
        let s = FixedString::<16>::from_str("test");
        let bytes = bytemuck::bytes_of(&s);
        let restored: &FixedString<16> =
            bytemuck::from_bytes(&bytes[..std::mem::size_of::<FixedString<16>>()]);
        assert_eq!(restored.as_str(), "test");
    }
}
