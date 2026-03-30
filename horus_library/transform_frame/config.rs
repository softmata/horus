//! Configuration for Transform Frame system

use super::types::MAX_SUPPORTED_FRAMES;

/// Configuration for Transform Frame system
///
/// Allows tuning memory usage vs capacity tradeoffs.
#[derive(Debug, Clone)]
pub struct TransformFrameConfig {
    /// Maximum number of frames (static + dynamic)
    ///
    /// Default: 256
    /// Range: 16 to 65536
    pub max_frames: usize,

    /// Maximum number of static frames
    ///
    /// Static frames are allocated first (IDs 0..max_static_frames).
    /// They use less memory since they don't need history buffers.
    ///
    /// Default: max_frames / 2
    pub max_static_frames: usize,

    /// History buffer length per dynamic frame
    ///
    /// Determines how many past transforms are kept for time-travel queries.
    /// Memory per dynamic frame = history_len * sizeof(TransformEntry) ≈ history_len * 64 bytes
    ///
    /// Default: 32
    /// Range: 4 to 256
    pub history_len: usize,

    /// Enable overflow to HashMap for unlimited frames
    ///
    /// When true, frames beyond max_frames fall back to a HashMap-based
    /// storage. This is slower but allows unlimited frames.
    ///
    /// Default: false (for predictable real-time performance)
    pub enable_overflow: bool,

    /// Cache size for transform chain lookups
    ///
    /// Caches recently computed src->dst transform chains.
    ///
    /// Default: 64
    pub chain_cache_size: usize,
}

impl Default for TransformFrameConfig {
    fn default() -> Self {
        Self::small()
    }
}

impl TransformFrameConfig {
    /// Small robot preset (256 frames, auto-grows if needed)
    ///
    /// Suitable for:
    /// - Single robots with <100 links
    /// - Simple sensor setups
    /// - Embedded systems
    ///
    /// Memory: ~50KB for slots + ~500KB for history = ~550KB
    /// (up to 4x more if overflow is triggered)
    ///
    /// `enable_overflow` is `true` by default: the registry auto-grows
    /// beyond 256 frames up to the core's physical capacity (1024) instead
    /// of returning an error. Disable overflow explicitly for hard real-time
    /// systems where predictable memory usage is required.
    pub fn small() -> Self {
        Self {
            max_frames: 256,
            max_static_frames: 128,
            history_len: 32,
            enable_overflow: true,
            chain_cache_size: 64,
        }
    }

    /// Medium robot preset (1024 frames, auto-grows if needed)
    ///
    /// Suitable for:
    /// - Complex robots (humanoids, manipulators)
    /// - Multi-robot setups (2-4 robots)
    /// - Desktop applications
    ///
    /// Memory: ~200KB for slots + ~2MB for history = ~2.2MB
    pub fn medium() -> Self {
        Self {
            max_frames: 1024,
            max_static_frames: 512,
            history_len: 32,
            enable_overflow: true,
            chain_cache_size: 128,
        }
    }

    /// Large simulation preset (4096 frames, auto-grows if needed)
    ///
    /// Suitable for:
    /// - Multi-robot simulations (10+ robots)
    /// - Complex environments with many objects
    /// - Development/testing scenarios
    ///
    /// Memory: ~800KB for slots + ~8MB for history = ~9MB
    pub fn large() -> Self {
        Self {
            max_frames: 4096,
            max_static_frames: 2048,
            history_len: 32,
            enable_overflow: true,
            chain_cache_size: 256,
        }
    }

    /// Massive simulation preset (16384 frames)
    ///
    /// Suitable for:
    /// - Large-scale multi-robot simulations (100+ robots)
    /// - Digital twin environments
    /// - Warehouse/factory simulations
    ///
    /// Memory: ~3.2MB for slots + ~32MB for history = ~35MB
    pub fn massive() -> Self {
        Self {
            max_frames: 16384,
            max_static_frames: 8192,
            history_len: 32,
            enable_overflow: true, // Allow overflow for safety
            chain_cache_size: 512,
        }
    }

    /// Unlimited frames preset (uses overflow)
    ///
    /// Suitable for:
    /// - Dynamic environments with unpredictable frame counts
    /// - Research/experimentation
    /// - Non-real-time applications
    ///
    /// Note: Not suitable for hard real-time due to HashMap allocation
    pub fn unlimited() -> Self {
        Self {
            max_frames: 4096, // Fast path capacity
            max_static_frames: 2048,
            history_len: 32,
            enable_overflow: true, // Key difference
            chain_cache_size: 512,
        }
    }

    /// Hard real-time preset (256 frames, no overflow)
    ///
    /// Use this when predictable memory usage is required and you know
    /// the exact number of frames your system will use.
    ///
    /// Returns an error instead of growing when the limit is reached.
    pub fn rt_fixed(max_frames: usize) -> Self {
        Self {
            max_frames,
            max_static_frames: max_frames / 2,
            history_len: 32,
            enable_overflow: false,
            chain_cache_size: 64,
        }
    }

    /// Custom configuration builder
    pub fn custom() -> TransformFrameConfigBuilder {
        TransformFrameConfigBuilder::new()
    }

    /// Validate configuration
    pub fn validate(&self) -> Result<(), String> {
        if self.max_frames < 16 {
            return Err("max_frames must be at least 16".to_string());
        }
        if self.max_frames > MAX_SUPPORTED_FRAMES {
            return Err(format!("max_frames cannot exceed {}", MAX_SUPPORTED_FRAMES));
        }
        if self.max_static_frames > self.max_frames {
            return Err("max_static_frames cannot exceed max_frames".to_string());
        }
        if self.history_len < 4 {
            return Err("history_len must be at least 4".to_string());
        }
        if self.history_len > 256 {
            return Err("history_len cannot exceed 256".to_string());
        }
        Ok(())
    }

    /// Calculate approximate memory usage in bytes
    pub fn estimated_memory_bytes(&self) -> usize {
        // Slot overhead (version, sequence, parent, flags, etc.)
        let slot_overhead = 64; // cache-line aligned

        // Transform entry size
        let transform_entry_size = 64; // 56 bytes + padding

        // Static frames: just slot overhead
        let static_memory = self.max_static_frames * slot_overhead;

        // Dynamic frames: slot + history buffer
        let dynamic_count = self.max_frames - self.max_static_frames;
        let dynamic_memory =
            dynamic_count * (slot_overhead + self.history_len * transform_entry_size);

        // Registry overhead (strings, hashmap)
        let registry_overhead = self.max_frames * 64; // Approximate

        // Chain cache
        let cache_memory = self.chain_cache_size * 128;

        static_memory + dynamic_memory + registry_overhead + cache_memory
    }

    /// Get human-readable memory estimate
    pub fn memory_estimate(&self) -> String {
        let bytes = self.estimated_memory_bytes();
        if bytes < 1024 {
            format!("{} bytes", bytes)
        } else if bytes < 1024 * 1024 {
            format!("{:.1} KB", bytes as f64 / 1024.0)
        } else {
            format!("{:.1} MB", bytes as f64 / (1024.0 * 1024.0))
        }
    }
}

/// Builder for custom TransformFrame configuration
pub struct TransformFrameConfigBuilder {
    config: TransformFrameConfig,
}

impl TransformFrameConfigBuilder {
    pub fn new() -> Self {
        Self {
            config: TransformFrameConfig::small(),
        }
    }

    /// Set maximum number of frames
    pub fn max_frames(mut self, n: usize) -> Self {
        self.config.max_frames = n;
        // Auto-adjust static frames if not explicitly set
        if self.config.max_static_frames > n {
            self.config.max_static_frames = n / 2;
        }
        self
    }

    /// Set maximum number of static frames
    pub fn max_static_frames(mut self, n: usize) -> Self {
        self.config.max_static_frames = n;
        self
    }

    /// Set history buffer length
    pub fn history_len(mut self, n: usize) -> Self {
        self.config.history_len = n;
        self
    }

    /// Enable overflow to HashMap for unlimited frames
    pub fn enable_overflow(mut self, enable: bool) -> Self {
        self.config.enable_overflow = enable;
        self
    }

    /// Set chain cache size
    pub fn chain_cache_size(mut self, n: usize) -> Self {
        self.config.chain_cache_size = n;
        self
    }

    /// Build and validate the configuration
    pub fn build(self) -> Result<TransformFrameConfig, String> {
        self.config.validate()?;
        Ok(self.config)
    }
}

impl Default for TransformFrameConfigBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_presets() {
        let small = TransformFrameConfig::small();
        assert_eq!(small.max_frames, 256);
        small.validate().unwrap();

        let medium = TransformFrameConfig::medium();
        assert_eq!(medium.max_frames, 1024);
        medium.validate().unwrap();

        let large = TransformFrameConfig::large();
        assert_eq!(large.max_frames, 4096);
        large.validate().unwrap();

        let massive = TransformFrameConfig::massive();
        assert_eq!(massive.max_frames, 16384);
        assert!(massive.enable_overflow);
        massive.validate().unwrap();
    }

    #[test]
    fn test_builder() {
        let config = TransformFrameConfig::custom()
            .max_frames(512)
            .history_len(64)
            .build()
            .unwrap();

        assert_eq!(config.max_frames, 512);
        assert_eq!(config.history_len, 64);
    }

    #[test]
    fn test_validation() {
        // Too few frames
        let mut config = TransformFrameConfig::small();
        config.max_frames = 8;
        assert!(config.validate().is_err());

        // Too many frames
        config.max_frames = 100_000;
        assert!(config.validate().is_err());

        // Static > max
        config = TransformFrameConfig::small();
        config.max_static_frames = 1000;
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_memory_estimate() {
        let small = TransformFrameConfig::small();
        let estimate = small.memory_estimate();
        assert!(estimate.contains("KB") || estimate.contains("MB"));

        let large = TransformFrameConfig::large();
        let estimate = large.memory_estimate();
        assert!(estimate.contains("MB"));
    }
}
