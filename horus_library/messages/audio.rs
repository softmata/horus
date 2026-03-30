// Audio message types for robotics
//
// Provides AudioFrame for microphone data, speech recognition,
// anomaly detection, and human-robot interaction.

use horus_core::core::LogSummary;
use serde::{Deserialize, Serialize};
use serde_arrays;

/// Maximum audio samples per frame.
///
/// Covers 48kHz sample rate at 100ms chunks (4800 samples).
/// For multi-channel audio, samples are interleaved (L R L R...).
pub const MAX_AUDIO_SAMPLES: usize = 4800;

/// Audio encoding format.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AudioEncoding {
    /// 32-bit float, range [-1.0, 1.0]
    F32 = 0,
    /// 16-bit signed integer, range [-32768, 32767]
    I16 = 1,
}

/// Audio data from a microphone or audio source.
///
/// Fixed-size buffer for zero-copy shared memory transport.
/// Use `num_samples` to indicate how many samples are valid.
///
/// # Example
/// ```rust,ignore
/// let frame = AudioFrame::mono(16000, &samples);
/// topic.send(frame);
/// ```
#[repr(C)]
#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct AudioFrame {
    /// Audio sample data (interleaved for multi-channel).
    #[serde(with = "serde_arrays")]
    pub samples: [f32; MAX_AUDIO_SAMPLES],
    /// Number of valid samples in the buffer.
    /// For stereo: num_samples = frames * 2.
    pub num_samples: u32,
    /// Sample rate in Hz (e.g., 8000, 16000, 44100, 48000).
    pub sample_rate: u32,
    /// Number of audio channels (1 = mono, 2 = stereo).
    pub channels: u8,
    /// Audio encoding format.
    pub encoding: u8,
    /// Padding for alignment.
    pub _pad: [u8; 2],
    /// Timestamp in nanoseconds since epoch.
    pub timestamp_ns: u64,
    /// Sensor/source identifier (e.g., "mic_left", "mic_array").
    pub frame_id: [u8; 32],
}

// Debug: don't print 4800 samples
impl std::fmt::Debug for AudioFrame {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AudioFrame")
            .field("num_samples", &self.num_samples)
            .field("sample_rate", &self.sample_rate)
            .field("channels", &self.channels)
            .field("duration_ms", &self.duration_ms())
            .finish()
    }
}

impl Default for AudioFrame {
    fn default() -> Self {
        Self {
            samples: [0.0; MAX_AUDIO_SAMPLES],
            num_samples: 0,
            sample_rate: 16000,
            channels: 1,
            encoding: AudioEncoding::F32 as u8,
            _pad: [0; 2],
            timestamp_ns: 0,
            frame_id: [0; 32],
        }
    }
}

impl AudioFrame {
    /// Create a mono audio frame from samples.
    ///
    /// Truncates to MAX_AUDIO_SAMPLES if input is longer.
    ///
    /// # Example
    /// ```rust,ignore
    /// let samples: Vec<f32> = mic.read_chunk();
    /// let frame = AudioFrame::mono(16000, &samples);
    /// topic.send(frame);
    /// ```
    pub fn mono(sample_rate: u32, samples: &[f32]) -> Self {
        let mut frame = Self {
            sample_rate,
            channels: 1,
            ..Default::default()
        };
        let count = samples.len().min(MAX_AUDIO_SAMPLES);
        frame.samples[..count].copy_from_slice(&samples[..count]);
        frame.num_samples = count as u32;
        frame
    }

    /// Create a stereo audio frame from interleaved samples (L R L R...).
    ///
    /// `samples` length should be even (pairs of left/right).
    pub fn stereo(sample_rate: u32, samples: &[f32]) -> Self {
        let mut frame = Self {
            sample_rate,
            channels: 2,
            ..Default::default()
        };
        let count = samples.len().min(MAX_AUDIO_SAMPLES);
        frame.samples[..count].copy_from_slice(&samples[..count]);
        frame.num_samples = count as u32;
        frame
    }

    /// Create a multi-channel audio frame.
    ///
    /// Samples are interleaved: [ch0_s0, ch1_s0, ch0_s1, ch1_s1, ...].
    pub fn multi_channel(sample_rate: u32, channels: u8, samples: &[f32]) -> Self {
        let mut frame = Self {
            sample_rate,
            channels,
            ..Default::default()
        };
        let count = samples.len().min(MAX_AUDIO_SAMPLES);
        frame.samples[..count].copy_from_slice(&samples[..count]);
        frame.num_samples = count as u32;
        frame
    }

    /// Set the frame ID (sensor/source name).
    pub fn with_frame_id(mut self, id: &str) -> Self {
        let bytes = id.as_bytes();
        let len = bytes.len().min(31);
        self.frame_id = [0; 32];
        self.frame_id[..len].copy_from_slice(&bytes[..len]);
        self
    }

    /// Set the timestamp.
    pub fn with_timestamp(mut self, timestamp_ns: u64) -> Self {
        self.timestamp_ns = timestamp_ns;
        self
    }

    /// Get valid samples as a slice.
    pub fn valid_samples(&self) -> &[f32] {
        let count = (self.num_samples as usize).min(MAX_AUDIO_SAMPLES);
        &self.samples[..count]
    }

    /// Duration of this frame in milliseconds.
    pub fn duration_ms(&self) -> f64 {
        if self.sample_rate == 0 || self.channels == 0 {
            return 0.0;
        }
        let frames = self.num_samples as f64 / self.channels as f64;
        (frames / self.sample_rate as f64) * 1000.0
    }

    /// Number of audio frames (samples per channel).
    pub fn frame_count(&self) -> u32 {
        if self.channels == 0 {
            return 0;
        }
        self.num_samples / self.channels as u32
    }

    /// Read the frame_id as a string.
    pub fn frame_id_str(&self) -> &str {
        let end = self.frame_id.iter().position(|&b| b == 0).unwrap_or(32);
        std::str::from_utf8(&self.frame_id[..end]).unwrap_or("")
    }
}

impl LogSummary for AudioFrame {
    fn log_summary(&self) -> String {
        format!(
            "audio: {}ch {}Hz {}samp {:.1}ms",
            self.channels,
            self.sample_rate,
            self.num_samples,
            self.duration_ms()
        )
    }
}

// Pod/Zeroable for zero-copy SHM
impl_pod_message!(AudioFrame);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mono_frame_basic() {
        let samples = vec![0.1, 0.2, 0.3, 0.4, 0.5];
        let frame = AudioFrame::mono(16000, &samples);
        assert_eq!(frame.num_samples, 5);
        assert_eq!(frame.sample_rate, 16000);
        assert_eq!(frame.channels, 1);
        assert_eq!(frame.valid_samples().len(), 5);
        assert_eq!(frame.valid_samples()[0], 0.1);
        assert_eq!(frame.frame_count(), 5);
    }

    #[test]
    fn stereo_frame_basic() {
        let samples = vec![0.1, 0.2, 0.3, 0.4]; // 2 frames, L R L R
        let frame = AudioFrame::stereo(48000, &samples);
        assert_eq!(frame.num_samples, 4);
        assert_eq!(frame.channels, 2);
        assert_eq!(frame.frame_count(), 2);
    }

    #[test]
    fn duration_calculation() {
        // 16000 samples at 16kHz mono = 1000ms
        let samples = vec![0.0; 1600];
        let frame = AudioFrame::mono(16000, &samples);
        assert!((frame.duration_ms() - 100.0).abs() < 0.01);
    }

    #[test]
    fn truncation_at_max() {
        let samples = vec![0.0; 10000]; // exceeds MAX_AUDIO_SAMPLES
        let frame = AudioFrame::mono(48000, &samples);
        assert_eq!(frame.num_samples as usize, MAX_AUDIO_SAMPLES);
    }

    #[test]
    fn frame_id_roundtrip() {
        let frame = AudioFrame::mono(16000, &[0.0]).with_frame_id("mic_left");
        assert_eq!(frame.frame_id_str(), "mic_left");
    }

    #[test]
    fn default_is_empty() {
        let frame = AudioFrame::default();
        assert_eq!(frame.num_samples, 0);
        assert_eq!(frame.valid_samples().len(), 0);
        assert_eq!(frame.duration_ms(), 0.0);
    }

    #[test]
    fn log_summary_format() {
        let frame = AudioFrame::mono(16000, &[0.0; 160]);
        let summary = frame.log_summary();
        assert!(summary.contains("1ch"));
        assert!(summary.contains("16000Hz"));
        assert!(summary.contains("160samp"));
    }

    #[test]
    fn multi_channel_mic_array() {
        let samples = vec![0.0; 64]; // 4 channels × 16 frames
        let frame = AudioFrame::multi_channel(16000, 4, &samples);
        assert_eq!(frame.channels, 4);
        assert_eq!(frame.frame_count(), 16);
    }

    #[test]
    fn zero_channels_no_panic() {
        let mut frame = AudioFrame::default();
        frame.channels = 0;
        assert_eq!(frame.duration_ms(), 0.0);
        assert_eq!(frame.frame_count(), 0);
    }
}
