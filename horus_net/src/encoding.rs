//! Encoding dispatch — handle cross-architecture byte order in the data path.
//!
//! See blueprint section 7. The `encoding` field in each message header declares
//! the byte order. Receiver checks if endianness matches and either memcpys
//! (zero overhead) or byte-swaps (rare path).

use crate::priority::Encoding;

/// Result of decoding payload bytes based on encoding tag.
#[derive(Debug)]
pub enum DecodeResult {
    /// Same endianness — payload bytes are usable as-is (zero-copy memcpy).
    SameEndian,
    /// Different endianness — payload bytes have been byte-swapped in place.
    ByteSwapped,
    /// Bincode-encoded — serde handles endianness.
    Bincode,
}

/// Check if the received encoding matches our native endianness.
///
/// Returns how the payload should be treated.
pub fn check_encoding(received: Encoding) -> DecodeResult {
    let native = Encoding::native_pod();
    match received {
        Encoding::Bincode => DecodeResult::Bincode,
        enc if enc == native => DecodeResult::SameEndian,
        _ => DecodeResult::ByteSwapped,
    }
}

/// Byte-swap a POD payload in-place.
///
/// This is the cross-endian path. It swaps bytes within each field boundary.
/// For simplicity, we do a generic byte-reversal of each N-byte word.
///
/// `word_size` is the size of each field (typically 4 for f32/u32, 8 for f64/u64).
/// For mixed-size structs, this is a best-effort approach — the common case
/// is uniform word sizes (all-f32 or all-f64 sensor data).
pub fn byte_swap_words(payload: &mut [u8], word_size: usize) {
    if word_size <= 1 || payload.is_empty() {
        return;
    }
    for chunk in payload.chunks_exact_mut(word_size) {
        chunk.reverse();
    }
}

/// Determine the word size to use for byte-swapping a POD type.
///
/// Heuristic: if type_size is divisible by 8, assume 8-byte words (f64).
/// If divisible by 4, assume 4-byte words (f32/u32).
/// Otherwise, assume 1 (no swap needed — single bytes).
pub fn infer_word_size(type_size: usize) -> usize {
    if type_size % 8 == 0 {
        8
    } else if type_size % 4 == 0 {
        4
    } else if type_size % 2 == 0 {
        2
    } else {
        1
    }
}

/// Process incoming payload: check encoding and byte-swap if needed.
///
/// Modifies `payload` in-place. Returns the decode result.
pub fn process_incoming_payload(
    payload: &mut [u8],
    encoding: Encoding,
    type_size: usize,
) -> DecodeResult {
    let result = check_encoding(encoding);
    if matches!(result, DecodeResult::ByteSwapped) {
        let word_size = infer_word_size(type_size);
        byte_swap_words(payload, word_size);
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_endian_is_noop() {
        let result = check_encoding(Encoding::native_pod());
        assert!(matches!(result, DecodeResult::SameEndian));
    }

    #[test]
    fn bincode_is_bincode() {
        let result = check_encoding(Encoding::Bincode);
        assert!(matches!(result, DecodeResult::Bincode));
    }

    #[test]
    fn cross_endian_detected() {
        // Force the "other" endian
        let other = if cfg!(target_endian = "little") {
            Encoding::PodBe
        } else {
            Encoding::PodLe
        };
        let result = check_encoding(other);
        assert!(matches!(result, DecodeResult::ByteSwapped));
    }

    #[test]
    fn byte_swap_4byte_words() {
        // Simulate a struct with two f32 fields: [1.0f32, 2.0f32]
        let original_a = 1.0f32.to_le_bytes();
        let original_b = 2.0f32.to_le_bytes();
        let mut payload = Vec::new();
        payload.extend_from_slice(&original_a);
        payload.extend_from_slice(&original_b);

        // Swap to big-endian
        byte_swap_words(&mut payload, 4);

        // Each 4-byte word should now be byte-reversed
        assert_eq!(&payload[0..4], &original_a.iter().rev().copied().collect::<Vec<_>>());
        assert_eq!(&payload[4..8], &original_b.iter().rev().copied().collect::<Vec<_>>());

        // Swap back
        byte_swap_words(&mut payload, 4);
        assert_eq!(&payload[0..4], &original_a);
        assert_eq!(&payload[4..8], &original_b);
    }

    #[test]
    fn byte_swap_8byte_words() {
        let val = 123456789.0f64;
        let mut payload = val.to_le_bytes().to_vec();
        let original = payload.clone();

        byte_swap_words(&mut payload, 8);
        assert_ne!(payload, original); // Changed

        byte_swap_words(&mut payload, 8);
        assert_eq!(payload, original); // Back to original
    }

    #[test]
    fn byte_swap_1byte_is_noop() {
        let mut payload = vec![1, 2, 3, 4];
        let original = payload.clone();
        byte_swap_words(&mut payload, 1);
        assert_eq!(payload, original);
    }

    #[test]
    fn byte_swap_empty_is_noop() {
        let mut payload: Vec<u8> = vec![];
        byte_swap_words(&mut payload, 4);
        assert!(payload.is_empty());
    }

    #[test]
    fn infer_word_size_f64() {
        // CmdVel: 16 bytes (2x f64) → 8
        assert_eq!(infer_word_size(16), 8);
        // Imu: 64 bytes → 8
        assert_eq!(infer_word_size(64), 8);
    }

    #[test]
    fn infer_word_size_f32() {
        // LaserScan ranges: 1440 bytes (360 × f32) → 8 (divisible by 8 too)
        // But 12 bytes (3 × f32) → 4
        assert_eq!(infer_word_size(12), 4);
    }

    #[test]
    fn process_incoming_same_endian() {
        let mut payload = vec![1, 2, 3, 4, 5, 6, 7, 8];
        let original = payload.clone();
        let result = process_incoming_payload(&mut payload, Encoding::native_pod(), 8);
        assert!(matches!(result, DecodeResult::SameEndian));
        assert_eq!(payload, original); // Unchanged
    }

    #[test]
    fn process_incoming_cross_endian_roundtrip() {
        // Simulate: sender is big-endian, we are little-endian (or vice versa)
        let val = 42.5f64;
        let mut payload = val.to_ne_bytes().to_vec();

        // Pretend we received from the "other" endian
        let other_encoding = if cfg!(target_endian = "little") {
            Encoding::PodBe
        } else {
            Encoding::PodLe
        };

        // First, simulate what the sender did: their ne bytes are our swapped bytes
        byte_swap_words(&mut payload, 8); // Now payload is in "other" endian

        // Process incoming: should detect mismatch and swap back
        process_incoming_payload(&mut payload, other_encoding, 8);

        // Should be back to our native encoding
        let decoded = f64::from_ne_bytes(payload.try_into().unwrap());
        assert_eq!(decoded, val);
    }
}
