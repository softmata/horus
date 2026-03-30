//! Fuzz target for shared memory header field interpretation.
//!
//! This exercises the raw byte-level parsing that `read_latest_slot_bytes`
//! performs on header fields, without file I/O overhead. It replicates the
//! exact offset arithmetic and bounds checking from header.rs to ensure
//! no combination of header field values causes panics or integer overflows.
//!
//! Header layout (640 bytes, #[repr(C, align(64))]):
//!   offset  0: magic (u64)       — must be 0x4144415054495645
//!   offset 12: type_size (u32)
//!   offset 20: is_pod (u8)       — 2 = POD, 1 = non-POD
//!   offset 64: write_idx (u64)   — sequence_or_head
//!   offset 72: capacity (u32)
//!   offset 76: cap_mask (u32)
//!   offset 80: slot_size (u32)

#![no_main]

use libfuzzer_sys::fuzz_target;

/// Magic value that marks a valid topic header.
const TOPIC_MAGIC: u64 = 0x4144415054495645;
/// Header size (10 cache lines).
const HEADER_SIZE: usize = 640;
/// POD flag value.
const POD_YES: u8 = 2;

/// Simulate the exact parsing logic from `read_latest_slot_bytes` in header.rs,
/// operating on an in-memory byte buffer instead of an mmap'd file.
fn parse_header_and_extract(data: &[u8], last_write_idx: u64) -> Option<Vec<u8>> {
    if data.len() < HEADER_SIZE {
        return None;
    }

    // Read magic (offset 0)
    let magic = u64::from_ne_bytes(data[0..8].try_into().ok()?);
    if magic != TOPIC_MAGIC {
        return None;
    }

    // Read header fields at exact offsets (matching #[repr(C, align(64))] layout)
    let type_size = u32::from_ne_bytes(data[12..16].try_into().ok()?) as usize;
    let is_pod_raw = data[20];
    let write_idx = u64::from_ne_bytes(data[64..72].try_into().ok()?);
    let capacity = u32::from_ne_bytes(data[72..76].try_into().ok()?) as usize;
    let cap_mask = u32::from_ne_bytes(data[76..80].try_into().ok()?) as usize;
    let slot_size = u32::from_ne_bytes(data[80..84].try_into().ok()?) as usize;

    let is_pod = is_pod_raw == POD_YES;

    // Guard: nothing written yet, or no new data since last poll
    if write_idx == 0 || write_idx == last_write_idx {
        return None;
    }

    // Index of the last-written slot
    let last_written = ((write_idx.wrapping_sub(1)) as usize) & cap_mask;

    if is_pod {
        if type_size == 0 || capacity == 0 {
            return None;
        }
        // Check for multiplication overflow
        let data_region = capacity.checked_mul(type_size)?;
        let required = HEADER_SIZE.checked_add(data_region)?;
        if data.len() < required {
            return None;
        }
        let slot_start = HEADER_SIZE.checked_add(last_written.checked_mul(type_size)?)?;
        let slot_end = slot_start.checked_add(type_size)?;
        if slot_end > data.len() {
            return None;
        }
        Some(data[slot_start..slot_end].to_vec())
    } else {
        if slot_size < 16 || capacity == 0 {
            return None;
        }
        let data_region = capacity.checked_mul(slot_size)?;
        let required = HEADER_SIZE.checked_add(data_region)?;
        if data.len() < required {
            return None;
        }
        let slot_start = HEADER_SIZE.checked_add(last_written.checked_mul(slot_size)?)?;
        let len_offset = slot_start.checked_add(8)?;
        let data_offset = slot_start.checked_add(16)?;
        if data_offset > data.len() || len_offset + 8 > data.len() {
            return None;
        }
        let data_len = u64::from_ne_bytes(data[len_offset..len_offset + 8].try_into().ok()?) as usize;
        if data_len == 0 {
            return None;
        }
        let end = data_offset.checked_add(data_len)?;
        if end > data.len() {
            return None;
        }
        Some(data[data_offset..end].to_vec())
    }
}

fuzz_target!(|data: &[u8]| {
    // Exercise the header parsing with various last_write_idx values
    let _ = parse_header_and_extract(data, 0);
    let _ = parse_header_and_extract(data, 1);
    let _ = parse_header_and_extract(data, u64::MAX);

    // Also try with the magic already set (to exercise the body parsing more often)
    if data.len() >= HEADER_SIZE {
        let mut modified = data.to_vec();
        modified[0..8].copy_from_slice(&TOPIC_MAGIC.to_ne_bytes());
        let _ = parse_header_and_extract(&modified, 0);
        let _ = parse_header_and_extract(&modified, 1);
    }
});
