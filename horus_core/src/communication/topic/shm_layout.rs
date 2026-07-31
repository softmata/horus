//! The topic SHM wire layout, as a single authoritative definition.
//!
//! # Why this module exists
//!
//! `TopicHeader` is `pub(crate)`, so anything outside `horus_core` that needs to
//! read or write a topic's backing file has to know its byte layout by other
//! means. `horus_net` did that by copying a table of offsets into a comment
//! (*"from header.rs layout analysis"*) and hard-coding them. The copy then
//! rotted, silently and comprehensively:
//!
//! | what | `horus_core` | the `horus_net` copy |
//! |---|---|---|
//! | data region start | `640 + capacity*8` | `640` — the seq array was omitted |
//! | `messages_total` | offset 56 | offset 48 — which is `topic_kind` |
//! | serde slot length | `slot + 8` | `slot + 0` — which is the ready word |
//! | per-slot ready flag | published every send | never written |
//!
//! Every one of those is a silent corruption of a live IPC region, and the
//! writer in question is fed by **unauthenticated network data**. Copied layout
//! knowledge with no compile-time link to the struct it describes is the root
//! cause, so this module fixes that rather than just the offsets: each constant
//! is tied to the real field by an `offset_of!` assertion in [`static_asserts`],
//! which is a `const` block — reordering a `TopicHeader` field is now a **build
//! failure**, not a four-month-old data corruption bug.
//!
//! # Region layout
//!
//! ```text
//! [ HEADER: 640 bytes ] [ SEQ_ARRAY: capacity * 8 ] [ DATA: capacity * stride ]
//! ```
//!
//! `stride` is `type_size` for POD topics and `slot_size` for serde topics.
//!
//! Serde slot layout: `[ 8B ready | 8B length | data… ]`.
//!
//! POD slots have no in-slot header; readiness lives in `SEQ_ARRAY`.
//!
//! # Readiness lives in two different places
//!
//! This trips people up, so it is stated plainly: the per-slot ready word that a
//! multi-producer consumer gates on is
//!
//!   * for **POD** topics, in `SEQ_ARRAY` at [`seq_slot_offset`] — see
//!     `dispatch::send_shm_sp_pod`;
//!   * for **serde** topics, the first 8 bytes *of the slot itself* at
//!     [`SERDE_SLOT_READY_OFF`] — see `dispatch::send_shm_mp_serde`.
//!
//! Both store `sequence + 1`. A writer that publishes only
//! `sequence_or_head` is visible to `SpscShm` consumers but invisible to
//! `MpscShm` ones, which is silent message loss rather than a loud failure.

use super::header::{TopicHeader, POD_YES, TOPIC_HEADER_SIZE, TOPIC_MAGIC};

// ─── Header constants, re-exported for out-of-crate consumers ───────────────

/// Magic number at offset 0 of every topic region ("ADAPTIVE").
pub const MAGIC: u64 = TOPIC_MAGIC;

/// Size of the fixed header that precedes the sequence array.
pub const HEADER_SIZE: usize = TOPIC_HEADER_SIZE;

/// Value of the `is_pod` byte meaning "this type is POD".
pub const IS_POD_YES: u8 = POD_YES;

// ─── Field offsets within the header ────────────────────────────────────────

/// `magic: u64`
pub const OFF_MAGIC: usize = 0;
/// `type_size: u32`
pub const OFF_TYPE_SIZE: usize = 12;
/// `is_pod: AtomicU8`
pub const OFF_IS_POD: usize = 20;
/// `topic_kind: u8`
pub const OFF_TOPIC_KIND: usize = 48;
/// `messages_total: AtomicU64`
pub const OFF_MESSAGES_TOTAL: usize = 56;
/// `sequence_or_head: AtomicU64` — the producer's publish point.
pub const OFF_SEQUENCE_OR_HEAD: usize = 64;
/// `capacity: u32`
pub const OFF_CAPACITY: usize = 72;
/// `capacity_mask: u32`
pub const OFF_CAPACITY_MASK: usize = 76;
/// `slot_size: u32`
pub const OFF_SLOT_SIZE: usize = 80;

// ─── Serde slot layout ──────────────────────────────────────────────────────

/// Offset of the per-slot ready word within a serde slot.
pub const SERDE_SLOT_READY_OFF: usize = 0;
/// Offset of the payload length (`u64`) within a serde slot.
pub const SERDE_SLOT_LEN_OFF: usize = 8;
/// Offset of the payload itself within a serde slot.
pub const SERDE_SLOT_DATA_OFF: usize = 16;
/// Bytes of per-slot overhead ahead of the payload in a serde slot.
pub const SERDE_SLOT_OVERHEAD: usize = SERDE_SLOT_DATA_OFF;

/// High bit of a per-slot ready stamp, set while a producer is writing that slot.
///
/// Part of the **shared** ring protocol, not an internal of any one backend:
/// `PodShm` broadcast overwrites without backpressure, so a consumer can be
/// mid-copy when a producer laps onto the same slot. The marker is the "writing"
/// phase of a Boehm seqlock — set before the data write, cleared by the done
/// stamp after it — and a consumer's re-check is **insufficient without it**,
/// because a producer that has started writing has not yet bumped the stamp.
///
/// EVERY writer into a topic ring must honour this, including out-of-crate ones
/// (`horus_net`'s `ShmRingWriter`). A writer that skips it reopens the torn-read
/// window for readers that are otherwise correct.
///
/// A high bit rather than a `pos << 1` tagging because stamps are compared
/// against `seq + 1` by the MpscShm and SpscShm paths and topics migrate between
/// backends; sequence numbers are u64 counters that will not reach 2^63.
pub const SLOT_WRITING: u64 = 1 << 63;

// ─── Derived geometry ───────────────────────────────────────────────────────

/// Byte offset of the per-slot ready-flag array (`capacity` × `u64`).
///
/// The array sits between the header and the data region. Consumers under the
/// multi-producer backend gate on `seq_array[index] == tail + 1`, so a writer
/// that does not publish here produces data no `MpscShm` reader will ever see.
#[inline]
pub const fn seq_array_offset() -> usize {
    HEADER_SIZE
}

/// Byte offset of the ready word for slot `index`.
#[inline]
pub const fn seq_slot_offset(index: usize) -> usize {
    seq_array_offset() + index * core::mem::size_of::<u64>()
}

/// Byte offset where the data region begins, for a ring of `capacity` slots.
///
/// **This is the constant `horus_net` got wrong.** Omitting the sequence array
/// places every write `capacity * 8` bytes early — on top of the ready flags.
#[inline]
pub const fn data_region_offset(capacity: usize) -> usize {
    HEADER_SIZE + capacity * core::mem::size_of::<u64>()
}

/// Byte offset of data slot `index`, given the per-slot `stride`.
///
/// `stride` is `type_size` for POD topics, `slot_size` for serde topics.
#[inline]
pub const fn data_slot_offset(capacity: usize, index: usize, stride: usize) -> usize {
    data_region_offset(capacity) + index * stride
}

/// Total bytes a region needs for `capacity` slots of `stride` bytes.
#[inline]
pub const fn required_region_len(capacity: usize, stride: usize) -> usize {
    data_region_offset(capacity) + capacity * stride
}

// ─── Compile-time drift detection ───────────────────────────────────────────

/// Ties every constant above to the actual `TopicHeader` field it describes.
///
/// These are `const` assertions: if a field is reordered, resized, or its
/// padding changes, the crate stops compiling. That is the whole point — the
/// previous arrangement let a copy of these numbers drift for four months
/// without a single test noticing.
mod static_asserts {
    use super::*;
    use core::mem::offset_of;

    const _: () = {
        assert!(offset_of!(TopicHeader, magic) == OFF_MAGIC);
        assert!(offset_of!(TopicHeader, type_size) == OFF_TYPE_SIZE);
        assert!(offset_of!(TopicHeader, is_pod) == OFF_IS_POD);
        assert!(offset_of!(TopicHeader, topic_kind) == OFF_TOPIC_KIND);
        assert!(offset_of!(TopicHeader, messages_total) == OFF_MESSAGES_TOTAL);
        assert!(offset_of!(TopicHeader, sequence_or_head) == OFF_SEQUENCE_OR_HEAD);
        assert!(offset_of!(TopicHeader, capacity) == OFF_CAPACITY);
        assert!(offset_of!(TopicHeader, capacity_mask) == OFF_CAPACITY_MASK);
        assert!(offset_of!(TopicHeader, slot_size) == OFF_SLOT_SIZE);

        // `topic_kind` is a single byte followed by 7 bytes of padding; a
        // 64-bit store at its offset would clobber both. horus_net did exactly
        // that, believing offset 48 held `messages_total`.
        assert!(OFF_MESSAGES_TOTAL == OFF_TOPIC_KIND + 8);

        // The header must be a whole number of cache lines, and the producer's
        // publish word must start its own line (false-sharing invariant).
        assert!(HEADER_SIZE == 640);
        assert!(OFF_SEQUENCE_OR_HEAD == 64);
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn data_region_accounts_for_the_sequence_array() {
        // capacity 8 → 64 bytes of ready flags between header and data.
        assert_eq!(seq_array_offset(), 640);
        assert_eq!(data_region_offset(8), 640 + 64);
        // The regression this module exists to prevent: data must NOT start at
        // HEADER_SIZE.
        assert_ne!(data_region_offset(8), HEADER_SIZE);
    }

    #[test]
    fn seq_slots_are_eight_bytes_apart_and_precede_the_data() {
        assert_eq!(seq_slot_offset(0), 640);
        assert_eq!(seq_slot_offset(1), 648);
        let capacity = 16;
        assert!(seq_slot_offset(capacity - 1) < data_region_offset(capacity));
    }

    #[test]
    fn data_slots_stride_correctly() {
        let capacity = 4;
        let stride = 24;
        let base = data_region_offset(capacity);
        assert_eq!(data_slot_offset(capacity, 0, stride), base);
        assert_eq!(data_slot_offset(capacity, 3, stride), base + 72);
    }

    #[test]
    fn required_len_covers_the_last_slot() {
        let (capacity, stride) = (8, 64);
        let end = data_slot_offset(capacity, capacity - 1, stride) + stride;
        assert_eq!(required_region_len(capacity, stride), end);
    }

    #[test]
    fn serde_slot_layout_matches_the_dispatch_writer() {
        // dispatch.rs writes: len at slot+8, data at slot+16, ready word at slot+0.
        assert_eq!(SERDE_SLOT_READY_OFF, 0);
        assert_eq!(SERDE_SLOT_LEN_OFF, 8);
        assert_eq!(SERDE_SLOT_DATA_OFF, 16);
        assert_eq!(SERDE_SLOT_OVERHEAD, 16);
    }

    #[test]
    fn zero_capacity_is_degenerate_but_total_order_holds() {
        // Guards against a divide/underflow style mistake in callers: with no
        // slots, the data region starts immediately after the header.
        assert_eq!(data_region_offset(0), HEADER_SIZE);
        assert_eq!(required_region_len(0, 32), HEADER_SIZE);
    }
}
