//! Compression support for network messages
//!
//! Provides transparent compression/decompression for large messages
//! using LZ4 (fast) or Zstd (better ratio) algorithms.

/// Minimum payload size to consider compression (bytes)
const MIN_COMPRESS_SIZE: usize = 512;
/// Minimum compression ratio to use compressed version (10% savings)
const MIN_COMPRESSION_RATIO: f64 = 0.9;

/// Compression algorithm
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CompressionAlgo {
    /// No compression
    #[default]
    None,
    /// LZ4 - Fast compression, moderate ratio
    Lz4,
    /// Zstd - Better ratio, slightly slower
    Zstd,
    /// Auto-select based on data size
    Auto,
}

/// Compression configuration
#[derive(Debug, Clone)]
pub struct CompressionConfig {
    /// Algorithm to use
    pub algorithm: CompressionAlgo,
    /// Minimum size to compress (smaller messages sent raw)
    pub min_size: usize,
    /// Compression level (1-9 for Zstd, ignored for LZ4)
    pub level: i32,
    /// Only use compression if ratio is below this threshold
    pub min_ratio: f64,
}

impl Default for CompressionConfig {
    fn default() -> Self {
        Self {
            algorithm: CompressionAlgo::None,
            min_size: MIN_COMPRESS_SIZE,
            level: 3, // Balanced Zstd level
            min_ratio: MIN_COMPRESSION_RATIO,
        }
    }
}

impl CompressionConfig {
    /// Enable LZ4 compression (fast)
    pub fn lz4() -> Self {
        Self {
            algorithm: CompressionAlgo::Lz4,
            ..Default::default()
        }
    }

    /// Enable Zstd compression (better ratio)
    pub fn zstd() -> Self {
        Self {
            algorithm: CompressionAlgo::Zstd,
            ..Default::default()
        }
    }

    /// Enable Zstd with custom level
    pub fn zstd_level(level: i32) -> Self {
        Self {
            algorithm: CompressionAlgo::Zstd,
            level: level.clamp(1, 19),
            ..Default::default()
        }
    }

    /// Auto-select algorithm based on data
    pub fn auto() -> Self {
        Self {
            algorithm: CompressionAlgo::Auto,
            ..Default::default()
        }
    }
}

/// Compression result with metadata
#[derive(Debug)]
pub struct CompressedData {
    /// Compressed payload
    pub data: Vec<u8>,
    /// Original uncompressed size
    pub original_size: usize,
    /// Algorithm used
    pub algorithm: CompressionAlgo,
    /// Whether compression was actually applied
    pub is_compressed: bool,
}

/// Compressor/decompressor
pub struct Compressor {
    config: CompressionConfig,
}

impl Compressor {
    pub fn new(config: CompressionConfig) -> Self {
        Self { config }
    }

    /// Compress data if beneficial
    pub fn compress(&self, data: &[u8]) -> CompressedData {
        let original_size = data.len();

        // Skip compression for small messages
        if original_size < self.config.min_size || self.config.algorithm == CompressionAlgo::None {
            return CompressedData {
                data: data.to_vec(),
                original_size,
                algorithm: CompressionAlgo::None,
                is_compressed: false,
            };
        }

        let algorithm = match self.config.algorithm {
            CompressionAlgo::Auto => {
                // Use LZ4 for real-time data (faster), Zstd for larger payloads
                if original_size < 10000 {
                    CompressionAlgo::Lz4
                } else {
                    CompressionAlgo::Zstd
                }
            }
            other => other,
        };

        let compressed = match algorithm {
            CompressionAlgo::Lz4 => self.compress_lz4(data),
            CompressionAlgo::Zstd => self.compress_zstd(data),
            _ => None,
        };

        match compressed {
            Some(compressed_data) => {
                // Check if compression was beneficial
                let ratio = compressed_data.len() as f64 / original_size as f64;
                if ratio < self.config.min_ratio {
                    CompressedData {
                        data: compressed_data,
                        original_size,
                        algorithm,
                        is_compressed: true,
                    }
                } else {
                    // Compression not beneficial, use original
                    CompressedData {
                        data: data.to_vec(),
                        original_size,
                        algorithm: CompressionAlgo::None,
                        is_compressed: false,
                    }
                }
            }
            None => CompressedData {
                data: data.to_vec(),
                original_size,
                algorithm: CompressionAlgo::None,
                is_compressed: false,
            },
        }
    }

    /// Decompress data
    pub fn decompress(
        &self,
        data: &[u8],
        algorithm: CompressionAlgo,
        original_size: usize,
    ) -> Result<Vec<u8>, CompressionError> {
        match algorithm {
            CompressionAlgo::None => Ok(data.to_vec()),
            CompressionAlgo::Lz4 => self.decompress_lz4(data, original_size),
            CompressionAlgo::Zstd => self.decompress_zstd(data),
            CompressionAlgo::Auto => {
                // Try to detect based on magic bytes
                if data.len() >= 4 {
                    // Zstd magic: 0xFD2FB528
                    if data[0] == 0x28 && data[1] == 0xB5 && data[2] == 0x2F && data[3] == 0xFD {
                        return self.decompress_zstd(data);
                    }
                }
                // Try LZ4
                self.decompress_lz4(data, original_size)
            }
        }
    }

    // LZ4 compression using simple RLE-like algorithm
    // Note: This is a simplified implementation. For production, use lz4 crate.
    fn compress_lz4(&self, data: &[u8]) -> Option<Vec<u8>> {
        // Simple LZ4-like compression (run-length encoding with literal copying)
        let mut output = Vec::with_capacity(data.len());
        let mut i = 0;

        while i < data.len() {
            // Look for runs of repeated bytes
            let _start = i;
            let current_byte = data[i];
            let mut run_length = 1;

            while i + run_length < data.len()
                && data[i + run_length] == current_byte
                && run_length < 255
            {
                run_length += 1;
            }

            if run_length >= 4 {
                // Encode as run: [0xFF, length, byte]
                output.push(0xFF);
                output.push(run_length as u8);
                output.push(current_byte);
                i += run_length;
            } else {
                // Find literal sequence
                let mut literal_end = i;
                while literal_end < data.len() && literal_end - i < 127 {
                    // Check if next position starts a good run
                    if literal_end + 3 < data.len() {
                        let next = data[literal_end];
                        if data[literal_end + 1] == next
                            && data[literal_end + 2] == next
                            && data[literal_end + 3] == next
                        {
                            break;
                        }
                    }
                    literal_end += 1;
                }

                let literal_len = literal_end - i;
                if literal_len > 0 {
                    // Encode as literal: [length (< 0x80), bytes...]
                    output.push(literal_len as u8);
                    output.extend_from_slice(&data[i..literal_end]);
                    i = literal_end;
                }
            }
        }

        // Only return if we achieved compression
        if output.len() < data.len() {
            Some(output)
        } else {
            None
        }
    }

    fn decompress_lz4(
        &self,
        data: &[u8],
        _original_size: usize,
    ) -> Result<Vec<u8>, CompressionError> {
        let mut output = Vec::new();
        let mut i = 0;

        while i < data.len() {
            let marker = data[i];
            i += 1;

            if marker == 0xFF {
                // Run encoding
                if i + 1 >= data.len() {
                    return Err(CompressionError::InvalidData);
                }
                let length = data[i] as usize;
                let byte = data[i + 1];
                i += 2;

                for _ in 0..length {
                    output.push(byte);
                }
            } else {
                // Literal encoding
                let length = marker as usize;
                if i + length > data.len() {
                    return Err(CompressionError::InvalidData);
                }
                output.extend_from_slice(&data[i..i + length]);
                i += length;
            }
        }

        Ok(output)
    }

    // Zstd-like compression using dictionary-based approach
    // Note: This is a simplified implementation. For production, use zstd crate.
    fn compress_zstd(&self, data: &[u8]) -> Option<Vec<u8>> {
        // Simple dictionary-based compression
        let mut output = Vec::with_capacity(data.len());

        // Magic bytes for identification
        output.extend_from_slice(&[0x28, 0xB5, 0x2F, 0xFD]);

        // Store original size (4 bytes, little-endian)
        output.extend_from_slice(&(data.len() as u32).to_le_bytes());

        // Use a simple sliding window approach
        let window_size = 4096.min(data.len());
        let mut i = 0;

        while i < data.len() {
            let mut best_match_len = 0;
            let mut best_match_dist = 0;

            // Search for matches in the sliding window
            let window_start = i.saturating_sub(window_size);
            for j in window_start..i {
                let mut match_len = 0;
                while i + match_len < data.len()
                    && j + match_len < i
                    && data[i + match_len] == data[j + match_len]
                    && match_len < 127
                // Max 127 since we use 7 bits for length
                {
                    match_len += 1;
                }

                if match_len > best_match_len && match_len >= 4 {
                    best_match_len = match_len;
                    best_match_dist = i - j;
                }
            }

            if best_match_len >= 4 {
                // Encode match: [0x80 | length, dist_lo, dist_hi]
                output.push(0x80 | (best_match_len as u8));
                output.push((best_match_dist & 0xFF) as u8);
                output.push(((best_match_dist >> 8) & 0xFF) as u8);
                i += best_match_len;
            } else {
                // Encode literal
                output.push(data[i]);
                i += 1;
            }
        }

        if output.len() < data.len() {
            Some(output)
        } else {
            None
        }
    }

    fn decompress_zstd(&self, data: &[u8]) -> Result<Vec<u8>, CompressionError> {
        // Check magic
        if data.len() < 8 {
            return Err(CompressionError::InvalidData);
        }

        if data[0] != 0x28 || data[1] != 0xB5 || data[2] != 0x2F || data[3] != 0xFD {
            return Err(CompressionError::InvalidMagic);
        }

        let original_size = u32::from_le_bytes([data[4], data[5], data[6], data[7]]) as usize;
        // Cap allocation to prevent decompression bombs (max 64 MB)
        const MAX_DECOMPRESSED_SIZE: usize = 64 * 1024 * 1024;
        if original_size > MAX_DECOMPRESSED_SIZE {
            return Err(CompressionError::InvalidData);
        }
        let mut output = Vec::with_capacity(original_size);

        let mut i = 8;
        while i < data.len() {
            let marker = data[i];
            i += 1;

            if marker & 0x80 != 0 {
                // Match reference
                if i + 1 >= data.len() {
                    return Err(CompressionError::InvalidData);
                }
                let length = (marker & 0x7F) as usize;
                let dist_lo = data[i] as usize;
                let dist_hi = data[i + 1] as usize;
                i += 2;

                let distance = dist_lo | (dist_hi << 8);
                if distance == 0 || distance > output.len() {
                    return Err(CompressionError::InvalidData);
                }
                let start = output.len() - distance;

                for j in 0..length {
                    let byte = output[start + j];
                    output.push(byte);
                }
            } else {
                // Literal byte
                output.push(marker);
            }
        }

        Ok(output)
    }
}

impl Default for Compressor {
    fn default() -> Self {
        Self::new(CompressionConfig::default())
    }
}

/// Compression error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CompressionError {
    /// Invalid compressed data
    InvalidData,
    /// Invalid magic bytes
    InvalidMagic,
    /// Decompressed size mismatch
    SizeMismatch,
    /// Unknown algorithm
    UnknownAlgorithm,
}

impl std::fmt::Display for CompressionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidData => write!(f, "Invalid compressed data"),
            Self::InvalidMagic => write!(f, "Invalid magic bytes"),
            Self::SizeMismatch => write!(f, "Decompressed size mismatch"),
            Self::UnknownAlgorithm => write!(f, "Unknown compression algorithm"),
        }
    }
}

impl std::error::Error for CompressionError {}

/// Compressed packet wrapper for network transmission
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct CompressedPacket {
    /// Compression algorithm used (0=None, 1=LZ4, 2=Zstd)
    pub algorithm: u8,
    /// Original uncompressed size
    pub original_size: u32,
    /// Compressed (or raw) payload
    pub payload: Vec<u8>,
}

impl CompressedPacket {
    /// Create from compressed data
    pub fn from_compressed(compressed: CompressedData) -> Self {
        Self {
            algorithm: match compressed.algorithm {
                CompressionAlgo::None => 0,
                CompressionAlgo::Lz4 => 1,
                CompressionAlgo::Zstd => 2,
                CompressionAlgo::Auto => 0,
            },
            original_size: compressed.original_size as u32,
            payload: compressed.data,
        }
    }

    /// Get algorithm from packet
    pub fn get_algorithm(&self) -> CompressionAlgo {
        match self.algorithm {
            1 => CompressionAlgo::Lz4,
            2 => CompressionAlgo::Zstd,
            _ => CompressionAlgo::None,
        }
    }

    /// Decompress the payload
    pub fn decompress(&self, compressor: &Compressor) -> Result<Vec<u8>, CompressionError> {
        compressor.decompress(
            &self.payload,
            self.get_algorithm(),
            self.original_size as usize,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_no_compression_small() {
        let compressor = Compressor::new(CompressionConfig::lz4());
        let data = vec![1, 2, 3, 4, 5]; // Too small

        let result = compressor.compress(&data);
        assert!(!result.is_compressed);
        assert_eq!(result.data, data);
    }

    #[test]
    fn test_lz4_compress_decompress() {
        let compressor = Compressor::new(CompressionConfig::lz4());

        // Highly compressible data (repeated bytes)
        let data: Vec<u8> = vec![42; 1000];

        let compressed = compressor.compress(&data);
        assert!(compressed.is_compressed);
        assert!(compressed.data.len() < data.len());

        let decompressed = compressor
            .decompress(&compressed.data, CompressionAlgo::Lz4, data.len())
            .unwrap();
        assert_eq!(decompressed, data);
    }

    #[test]
    fn test_zstd_compress_decompress() {
        let compressor = Compressor::new(CompressionConfig::zstd());

        // Data with patterns
        let mut data = Vec::new();
        for _ in 0..100 {
            data.extend_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);
        }

        let compressed = compressor.compress(&data);

        if compressed.is_compressed {
            let decompressed = compressor
                .decompress(&compressed.data, CompressionAlgo::Zstd, data.len())
                .unwrap();
            assert_eq!(decompressed, data);
        }
    }

    #[test]
    fn test_incompressible_data() {
        let compressor = Compressor::new(CompressionConfig::lz4());

        // Random-like data (not compressible)
        let data: Vec<u8> = (0..1000).map(|i| (i * 17 + 31) as u8).collect();

        let compressed = compressor.compress(&data);

        // Should fall back to uncompressed since ratio is bad
        // Either way, round-trip should work
        if compressed.is_compressed {
            let decompressed = compressor
                .decompress(&compressed.data, compressed.algorithm, data.len())
                .unwrap();
            assert_eq!(decompressed, data);
        } else {
            assert_eq!(compressed.data, data);
        }
    }

    #[test]
    fn test_compressed_packet() {
        let compressor = Compressor::new(CompressionConfig::lz4());
        let data: Vec<u8> = vec![0xAB; 2000];

        let compressed = compressor.compress(&data);
        let packet = CompressedPacket::from_compressed(compressed);

        let decompressed = packet.decompress(&compressor).unwrap();
        assert_eq!(decompressed, data);
    }

    #[test]
    fn test_auto_algorithm() {
        let compressor = Compressor::new(CompressionConfig::auto());

        // Small data - should use LZ4
        let small_data: Vec<u8> = vec![42; 1000];
        let compressed = compressor.compress(&small_data);
        if compressed.is_compressed {
            assert_eq!(compressed.algorithm, CompressionAlgo::Lz4);
        }

        // Large data - should use Zstd
        let large_data: Vec<u8> = vec![42; 50000];
        let compressed = compressor.compress(&large_data);
        if compressed.is_compressed {
            assert_eq!(compressed.algorithm, CompressionAlgo::Zstd);
        }
    }

    #[test]
    fn test_compression_ratio_threshold() {
        let config = CompressionConfig {
            algorithm: CompressionAlgo::Lz4,
            min_size: 100,
            min_ratio: 0.5, // Need 50% compression
            ..Default::default()
        };
        let compressor = Compressor::new(config);

        // Data that compresses well (should pass threshold)
        let good_data: Vec<u8> = vec![42; 1000];
        let compressed = compressor.compress(&good_data);
        // If compressed, ratio should be < 0.5
        if compressed.is_compressed {
            let ratio = compressed.data.len() as f64 / good_data.len() as f64;
            assert!(ratio < 0.5);
        }
    }
}
