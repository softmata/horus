//! Internal Pod image descriptor for zero-copy ring buffer transport
//!
//! `ImageDescriptor` is a fixed-size (224 byte) `repr(C)` descriptor that flows
//! through the ring buffer via the ~50ns Pod path. Actual pixel data lives in a
//! `TensorPool` — only the descriptor is copied.
//!
//! Users should use `Image` from `horus_core` which wraps this with data access.

use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};

use super::image_encoding::ImageEncoding;
use super::tensor::Tensor;

/// Unified image descriptor — Pod, 224 bytes.
///
/// Contains a `Tensor` (shape `[H, W, C]`) plus domain metadata
/// (encoding, step, frame_id, timestamp). This is what flows through
/// the ring buffer; pixel data stays in shared memory.
///
/// # Layout (224 bytes, repr(C))
///
/// ```text
/// inner:        Tensor  (168 bytes)
/// timestamp_ns: u64          (8 bytes)
/// step:         u32          (4 bytes)
/// encoding_raw: u8           (1 byte, ImageEncoding discriminant)
/// _pad:         [u8; 3]      (3 bytes)
/// frame_id:     [u8; 32]     (32 bytes)
/// capture_ns:   u64          (8 bytes)
/// Total:                      224 bytes
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct ImageDescriptor {
    /// Inner tensor: shape [H, W, C], data in pool
    inner: Tensor,
    /// Timestamp in nanoseconds since epoch
    timestamp_ns: u64,
    /// Bytes per row (may include padding for alignment)
    step: u32,
    /// Pixel encoding format, held as the raw `repr(u8)` discriminant byte.
    ///
    /// This was `encoding: ImageEncoding`, and it was unsound: the descriptor is
    /// read byte-for-byte out of peer-writable shared memory, so a byte outside
    /// `0..=ImageEncoding::MAX_DISCRIMINANT` materialised an invalid enum
    /// discriminant — undefined behaviour committed *before* any sanitiser could
    /// run.  Read it through [`ImageDescriptor::encoding`], which launders the
    /// byte via `ImageEncoding::from_raw`.  Same offset and size, so the
    /// 224-byte wire layout is unchanged; `serde` still names the field
    /// `encoding` and still writes the enum form.
    #[serde(rename = "encoding", with = "encoding_serde")]
    encoding_raw: u8,
    #[serde(skip)]
    _pad: [u8; 3],
    /// Frame ID (camera identifier, null-terminated)
    frame_id: [u8; 32],
    /// Instant the sensor SAMPLED this frame, nanoseconds since the UNIX epoch.
    /// `0` means unknown. See [`impl_capture_ns_field`] for why this is not
    /// `timestamp_ns`.
    ///
    /// Occupies what were 8 reserved bytes, so the wire layout is unchanged and
    /// a peer built before this reads them as the zeroed reserved bytes it
    /// already ignored. The topic type hash is derived from the type NAME
    /// (`fnv1a_type_hash(type_name::<T>())`), not the field layout, so old and
    /// new peers still bind. `serde(default)` keeps older recordings loadable.
    #[serde(default)]
    capture_ns: u64,
}

/// Serde bridge for the raw encoding byte.
///
/// The discriminant is stored as a `u8` (see `encoding_raw`), but the serde form
/// must stay byte-for-byte what it was when the field held an `ImageEncoding`,
/// so recordings and JSON written by older builds still load.
mod encoding_serde {
    use super::ImageEncoding;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    pub fn serialize<S: Serializer>(raw: &u8, serializer: S) -> Result<S::Ok, S::Error> {
        ImageEncoding::from_raw(*raw).serialize(serializer)
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(deserializer: D) -> Result<u8, D::Error> {
        Ok(ImageEncoding::deserialize(deserializer)? as u8)
    }
}

// Safety: Image is repr(C), every field is a plain integer or an array of them
// (the encoding discriminant is a raw `u8` precisely so that all 256 byte values
// really are valid, as `Pod` requires), no implicit padding.
// 168 + 8 + 4 + 1 + 3 + 32 + 8 = 224 bytes, 224 % 8 = 0.
unsafe impl Zeroable for ImageDescriptor {}
unsafe impl Pod for ImageDescriptor {}

impl Default for ImageDescriptor {
    fn default() -> Self {
        Self {
            inner: Tensor::default(),
            timestamp_ns: 0,
            step: 0,
            encoding_raw: ImageEncoding::Rgb8 as u8,
            _pad: [0; 3],
            frame_id: [0; 32],
            capture_ns: 0,
        }
    }
}

impl ImageDescriptor {
    /// Sanitize an ImageDescriptor read from untrusted bytes (SHM, network, file).
    ///
    /// Clamps inner tensor fields and normalises the raw encoding byte.
    /// [`encoding`](Self::encoding) already launders the byte on every read —
    /// this only makes the *stored* byte agree, so a descriptor forwarded on the
    /// wire carries the clamped value.
    #[inline]
    pub fn sanitize_from_shm(&mut self) {
        self.inner.sanitize_from_shm();
        self.encoding_raw = self.encoding() as u8;
    }

    /// Create a new image descriptor from pre-built tensor + metadata.
    ///
    /// The `step` (bytes per row) is taken from `tensor.strides[0]`, which
    /// reflects the actual row stride including any alignment padding.  This
    /// is correct for tensors allocated with row-padding (e.g., SIMD-aligned
    /// images).  Falls back to `width * bytes_per_pixel` only when the stride
    /// is zero (default/zeroed tensors).
    pub fn new(tensor: Tensor, encoding: ImageEncoding) -> Self {
        // Use the tensor's actual row stride (strides[0]) so that padded
        // images — where the row stride exceeds width * bpp — have their
        // step set correctly.  pixel() / set_pixel() / roi() all depend on
        // step being accurate; a wrong step causes them to read or write the
        // wrong bytes in adjacent rows.
        let step = if tensor.ndim >= 1 && tensor.strides[0] > 0 {
            tensor.strides[0] as u32
        } else {
            // Fallback for zeroed/default tensors (no meaningful stride).
            let width = if tensor.ndim >= 2 {
                tensor.shape[1] as u32
            } else {
                0
            };
            width * encoding.bytes_per_pixel()
        };

        Self {
            inner: tensor,
            timestamp_ns: 0,
            step,
            encoding_raw: encoding as u8,
            _pad: [0; 3],
            frame_id: [0; 32],
            capture_ns: 0,
        }
    }

    /// Wrap an existing `Tensor` as an `Image`, inferring encoding.
    pub fn from_tensor(tensor: Tensor) -> Self {
        let channels = if tensor.ndim >= 3 {
            tensor.shape[2] as u32
        } else {
            1
        };
        let encoding = ImageEncoding::from_dtype_channels(tensor.dtype(), channels);
        Self::new(tensor, encoding)
    }

    // === Metadata accessors ===

    /// Image height in pixels (tensor dimension 0).
    #[inline]
    pub fn height(&self) -> u32 {
        if self.inner.ndim >= 1 {
            self.inner.shape[0] as u32
        } else {
            0
        }
    }

    /// Image width in pixels (tensor dimension 1).
    #[inline]
    pub fn width(&self) -> u32 {
        if self.inner.ndim >= 2 {
            self.inner.shape[1] as u32
        } else {
            0
        }
    }

    /// Number of channels (tensor dimension 2, defaults to 1).
    #[inline]
    pub fn channels(&self) -> u32 {
        if self.inner.ndim >= 3 {
            self.inner.shape[2] as u32
        } else {
            1
        }
    }

    /// Pixel encoding format.
    #[inline]
    pub fn encoding(&self) -> ImageEncoding {
        ImageEncoding::from_raw(self.encoding_raw)
    }

    /// Bytes per row.
    #[inline]
    pub fn step(&self) -> u32 {
        self.step
    }

    crate::impl_tensor_accessors!();
    crate::impl_timestamp_field!();
    crate::impl_capture_ns_field!();
    crate::impl_frame_id_field!();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Device, TensorDtype};

    /// `ImageDescriptor` used to carry `encoding: ImageEncoding` under
    /// `unsafe impl Pod`, so a descriptor byte-copied out of peer-writable SHM
    /// could hold a discriminant no variant names — undefined behaviour the
    /// moment `bytes_per_pixel()` matched on it.  The byte is private and
    /// laundered now, and it must still sit at the same wire offset.
    #[test]
    fn an_invalid_encoding_byte_is_laundered_and_stays_at_its_wire_offset() {
        const ENCODING_BYTE_OFFSET: usize = 180; // 168 (Tensor) + 8 (ts) + 4 (step)

        let tensor = Tensor::new(1, 0, 0, 0, &[8, 8, 1], TensorDtype::U8, Device::cpu());
        let mut img = ImageDescriptor::new(tensor, ImageEncoding::Mono8);
        assert_eq!(
            bytemuck::bytes_of(&img)[ENCODING_BYTE_OFFSET],
            ImageEncoding::Mono8 as u8,
            "the encoding discriminant must stay at wire offset 180"
        );

        img.encoding_raw = 0xC0; // names no variant
        assert_eq!(
            img.encoding(),
            ImageEncoding::Rgb8,
            "a discriminant no variant names must read back as the Rgb8 fallback"
        );
        img.sanitize_from_shm();
        assert_eq!(
            bytemuck::bytes_of(&img)[ENCODING_BYTE_OFFSET],
            ImageEncoding::Rgb8 as u8
        );
    }

    #[test]
    fn test_image_size_and_alignment() {
        let size = std::mem::size_of::<ImageDescriptor>();
        let align = std::mem::align_of::<ImageDescriptor>();
        assert_eq!(size, 224, "ImageDescriptor must be exactly 224 bytes");
        // Alignment must be 8 (from u64 fields in inner Tensor)
        assert_eq!(align, 8);
        // Size is multiple of alignment (no trailing padding)
        assert_eq!(size % align, 0);
        // 224 = 32 * 7, divisible by common cache-line fractions
        assert_eq!(size % 32, 0);
    }

    #[test]
    fn test_image_pod() {
        // Verify Pod by roundtripping a configured descriptor through bytes
        let tensor = Tensor::new(1, 42, 99, 0, &[480, 640, 3], TensorDtype::U8, Device::cpu());
        let mut img = ImageDescriptor::new(tensor, ImageEncoding::Rgb8);
        img.set_frame_id("test_cam");
        img.set_timestamp_ns(1_000_000_000);

        let bytes: &[u8] = bytemuck::bytes_of(&img);
        assert_eq!(bytes.len(), 224);
        let recovered: &ImageDescriptor = bytemuck::from_bytes(bytes);
        assert_eq!(recovered.height(), 480);
        assert_eq!(recovered.width(), 640);
        assert_eq!(recovered.encoding(), ImageEncoding::Rgb8);
        assert_eq!(recovered.frame_id(), "test_cam");
        assert_eq!(recovered.timestamp_ns(), 1_000_000_000);
    }

    #[test]
    fn test_image_from_tensor() {
        let tensor = Tensor::new(1, 0, 0, 0, &[480, 640, 3], TensorDtype::U8, Device::cpu());
        let img = ImageDescriptor::from_tensor(tensor);
        assert_eq!(img.height(), 480);
        assert_eq!(img.width(), 640);
        assert_eq!(img.channels(), 3);
        assert_eq!(img.encoding(), ImageEncoding::Rgb8);
        assert_eq!(img.step(), 640 * 3);
    }

    #[test]
    fn test_image_mono() {
        let tensor = Tensor::new(1, 0, 0, 0, &[100, 200], TensorDtype::U8, Device::cpu());
        let img = ImageDescriptor::from_tensor(tensor);
        assert_eq!(img.channels(), 1);
        assert_eq!(img.encoding(), ImageEncoding::Mono8);
    }

    #[test]
    fn test_image_frame_id() {
        let mut img = ImageDescriptor::default();
        img.set_frame_id("camera_left");
        assert_eq!(img.frame_id(), "camera_left");
    }

    #[test]
    fn test_image_step_uses_tensor_stride() {
        // Create a tensor with a padded row stride (e.g., SIMD-aligned rows).
        // strides[0] = width * channels + 16 bytes of padding per row.
        let mut tensor = Tensor::new(1, 0, 0, 0, &[480, 640, 3], TensorDtype::U8, Device::cpu());
        let padded_stride = 640u64 * 3 + 16;
        tensor.strides[0] = padded_stride;

        let img = ImageDescriptor::new(tensor, ImageEncoding::Rgb8);

        // step must reflect the padded stride, not width * bpp.
        assert_eq!(
            img.step(),
            padded_stride as u32,
            "step should equal tensor.strides[0] for padded images"
        );
        assert_ne!(
            img.step(),
            640 * 3,
            "step must NOT equal width*bpp when there is row padding"
        );
    }

    #[test]
    fn test_image_step_fallback_for_zeroed_tensor() {
        // A zeroed (default) tensor has strides[0] == 0 — fallback to width*bpp.
        let tensor = Tensor::default(); // all zeros, ndim == 0
        let img = ImageDescriptor::new(tensor, ImageEncoding::Rgb8);
        // Both width and step should be 0 for an empty descriptor.
        assert_eq!(img.step(), 0);
    }

    #[test]
    fn test_image_serde_roundtrip() {
        let tensor = Tensor::new(
            1,
            42,
            1,
            0,
            &[1080, 1920, 3],
            TensorDtype::U8,
            Device::cpu(),
        );
        let mut img = ImageDescriptor::new(tensor, ImageEncoding::Rgb8);
        img.set_frame_id("cam0");
        img.set_timestamp_ns(123456789);

        let json = serde_json::to_string(&img).unwrap();
        let recovered: ImageDescriptor = serde_json::from_str(&json).unwrap();
        assert_eq!(recovered.height(), 1080);
        assert_eq!(recovered.width(), 1920);
        assert_eq!(recovered.encoding(), ImageEncoding::Rgb8);
        assert_eq!(recovered.frame_id(), "cam0");
        assert_eq!(recovered.timestamp_ns(), 123456789);
    }
}

#[cfg(test)]
mod capture_ns_tests {
    use super::*;

    /// The whole compatibility argument rests on the size not moving.
    ///
    /// `capture_ns` occupies what were 8 reserved bytes. If that ever stops
    /// being true, a peer built before the change and one built after disagree
    /// about the descriptor's length while still binding — the type hash is
    /// derived from the type NAME, not the layout, so nothing else would catch
    /// it. This is the thing that catches it.
    #[test]
    fn descriptor_is_still_224_bytes() {
        assert_eq!(
            std::mem::size_of::<ImageDescriptor>(),
            224,
            "ImageDescriptor changed size; capture_ns was supposed to reuse the \
             8 reserved bytes, and peers built either side of this change bind \
             on type name alone"
        );
    }

    /// Unset means unknown, and unknown must be zero — the value an older peer
    /// leaves in the bytes it treated as reserved.
    #[test]
    fn capture_ns_defaults_to_zero_meaning_unknown() {
        let d = ImageDescriptor::default();
        assert_eq!(d.capture_ns(), 0);
    }

    /// capture and publish are independent: setting one must not move the other.
    /// They are different instants and the entire point of the field is that a
    /// consumer can tell them apart.
    #[test]
    fn capture_and_publish_timestamps_are_independent() {
        let mut d = ImageDescriptor::default();
        d.set_timestamp_ns(1_000_000_000);
        d.set_capture_ns(970_000_000); // sampled 30 ms before publish
        assert_eq!(d.timestamp_ns(), 1_000_000_000);
        assert_eq!(d.capture_ns(), 970_000_000);

        d.set_timestamp_ns(2_000_000_000);
        assert_eq!(
            d.capture_ns(),
            970_000_000,
            "re-stamping publish time must not disturb the capture instant"
        );
    }

    /// A recording written before this field existed has no `capture_ns` key.
    /// It must still load, as unknown, rather than failing to deserialize.
    #[test]
    fn recordings_without_the_field_still_load() {
        let mut d = ImageDescriptor::default();
        d.set_timestamp_ns(42);
        d.set_capture_ns(7);
        let json = serde_json::to_string(&d).expect("serialize");
        assert!(
            json.contains("capture_ns"),
            "new recordings carry the field"
        );

        // Strip it, as an older writer would have produced.
        let mut v: serde_json::Value = serde_json::from_str(&json).expect("parse");
        v.as_object_mut().expect("object").remove("capture_ns");
        let legacy = serde_json::to_string(&v).expect("reserialize");

        let back: ImageDescriptor =
            serde_json::from_str(&legacy).expect("a legacy recording must still load");
        assert_eq!(back.timestamp_ns(), 42);
        assert_eq!(
            back.capture_ns(),
            0,
            "a recording that predates the field reports unknown, not garbage"
        );
    }
}
