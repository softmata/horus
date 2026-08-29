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
//! is tied to the real field by an `offset_of!` assertion in `static_asserts`,
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
//!   * for **POD** topics, in `SEQ_ARRAY` at `seq_slot_offset` — see
//!     `dispatch::send_shm_sp_pod`;
//!   * for **serde** topics, the first 8 bytes *of the slot itself* at
//!     `SERDE_SLOT_READY_OFF` — see `dispatch::send_shm_mp_serde`.
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

/// [`required_region_len`], refusing to wrap.
///
/// The unchecked form is fine for a geometry this process computed. It is NOT
/// fine for one read out of a shared header, where `capacity` and `stride` are
/// whatever another process stored: with `overflow-checks` off — the release
/// profile a robot ships — `capacity * stride` wraps and the total comes back
/// *smaller* than the header itself, so a containment check written against it
/// passes for a ring that does not fit in the mapping at all. Every caller
/// validating an attacker-controlled header must use this one.
#[inline]
pub fn required_region_len_checked(capacity: usize, stride: usize) -> Option<usize> {
    capacity
        .checked_mul(core::mem::size_of::<u64>())?
        .checked_add(capacity.checked_mul(stride)?)?
        .checked_add(HEADER_SIZE)
}

// ─── Co-located slot layout (POD, small types) ──────────────────────────────

/// `layout_kind` value: the historical split layout described above.
pub const LAYOUT_SPLIT: u8 = 0;
/// `layout_kind` value: stamp co-located with its payload, one slot per line.
pub const LAYOUT_COLO: u8 = 1;

/// `layout_kind: AtomicU8` — which of the two geometries this region uses.
///
/// Carved out of `_pad1a`, so `messages_total` stays at 56 and every offset
/// above is unchanged.
pub const OFF_LAYOUT_KIND: usize = 49;

/// A cache line. Colo slots are padded to a multiple of this so that no two
/// slots ever share one.
pub const CACHE_LINE: usize = 64;

/// Offset of the readiness stamp within a colo slot.
pub const COLO_STAMP_OFF: usize = 0;
/// Offset of the payload within a colo slot.
pub const COLO_PAYLOAD_OFF: usize = 8;

/// Largest payload that still shares its stamp's cache line.
///
/// Past this the payload spills onto a second line and the co-location win
/// largely disappears — measured at ~60ns for payloads at or under this bound
/// versus ~17ns at 64 bytes, on an i7-10750H. The bound is therefore the
/// eligibility rule, not a soft preference.
pub const COLO_MAX_PAYLOAD: usize = CACHE_LINE - COLO_PAYLOAD_OFF;

/// Whether a topic of `type_size` bytes should use the colo layout.
///
/// POD only: a serde topic carries its own in-slot length word and variable
/// payload, so there is no fixed geometry to co-locate.
#[inline]
pub const fn colo_eligible(is_pod: bool, type_size: usize) -> bool {
    is_pod && type_size > 0 && type_size <= COLO_MAX_PAYLOAD
}

/// Bytes per colo slot: stamp + payload, rounded up to whole cache lines.
///
/// The padding is load-bearing. It is what stops two slots sharing a line —
/// today's split layout strides small PODs at 64 bytes for allocation but
/// indexes them by `type_size`, so four 16-byte slots land in one line and
/// adjacent producers false-share.
#[inline]
pub const fn colo_slot_size(type_size: usize) -> usize {
    let raw = COLO_PAYLOAD_OFF + type_size;
    raw.div_ceil(CACHE_LINE) * CACHE_LINE
}

/// Byte offset of colo slot `index`. Colo has no separate sequence array, so
/// the slots begin immediately after the header.
#[inline]
pub const fn colo_slot_offset(index: usize, slot_size: usize) -> usize {
    HEADER_SIZE + index * slot_size
}

/// Byte offset of the stamp for colo slot `index`.
#[inline]
pub const fn colo_stamp_offset(index: usize, slot_size: usize) -> usize {
    colo_slot_offset(index, slot_size) + COLO_STAMP_OFF
}

/// Byte offset of the payload for colo slot `index`.
#[inline]
pub const fn colo_payload_offset(index: usize, slot_size: usize) -> usize {
    colo_slot_offset(index, slot_size) + COLO_PAYLOAD_OFF
}

/// Total bytes a colo region needs for `capacity` slots.
#[inline]
pub const fn colo_required_region_len(capacity: usize, slot_size: usize) -> usize {
    HEADER_SIZE + capacity * slot_size
}

/// [`colo_required_region_len`], refusing to wrap.
///
/// Same reason as [`required_region_len_checked`]: a colo geometry read out of
/// a shared header carries another process's `capacity` and `slot_size`, and a
/// containment check built on a wrapped product passes for a ring that does not
/// fit the mapping. Colo has no sequence array, so this is the shorter product
/// — which is exactly why the split form cannot be reused to validate it: it
/// would demand `capacity * 8` bytes that a colo region correctly does not have,
/// and reject every valid one.
#[inline]
pub fn colo_required_region_len_checked(capacity: usize, slot_size: usize) -> Option<usize> {
    capacity.checked_mul(slot_size)?.checked_add(HEADER_SIZE)
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
        assert!(offset_of!(TopicHeader, layout_kind) == OFF_LAYOUT_KIND);
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

        // Colo slots must start cache-line aligned, or every slot straddles
        // two lines and the layout is worse than the one it replaces.
        assert!(HEADER_SIZE.is_multiple_of(CACHE_LINE));
        // The stamp and a maximum-size payload must fit one line exactly.
        assert!(COLO_PAYLOAD_OFF + COLO_MAX_PAYLOAD == CACHE_LINE);
        // `layout_kind` lives inside what used to be `_pad1a`, between
        // `topic_kind` and `messages_total`, so it displaces no field.
        assert!(OFF_LAYOUT_KIND > OFF_TOPIC_KIND);
        assert!(OFF_LAYOUT_KIND < OFF_MESSAGES_TOTAL);
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
    fn the_checked_length_agrees_with_the_plain_one_when_nothing_wraps() {
        for (capacity, stride) in [(1usize, 8usize), (8, 64), (512, 4096)] {
            assert_eq!(
                required_region_len_checked(capacity, stride),
                Some(required_region_len(capacity, stride))
            );
        }
    }

    /// The case the unchecked form gets wrong: a geometry out of an untrusted
    /// header whose product wraps. `required_region_len` returns a SMALL number
    /// there, so `required <= mapped_len` passes and the caller maps a ring it
    /// cannot address.
    #[test]
    fn a_wrapping_geometry_is_refused_rather_than_reported_small() {
        let capacity = 1usize << 40;
        let stride = 1usize << 40;
        assert_eq!(required_region_len_checked(capacity, stride), None);
        // capacity alone can overflow the sequence array, before any stride.
        assert_eq!(required_region_len_checked(usize::MAX, 1), None);
    }

    #[test]
    fn zero_capacity_is_degenerate_but_total_order_holds() {
        // Guards against a divide/underflow style mistake in callers: with no
        // slots, the data region starts immediately after the header.
        assert_eq!(data_region_offset(0), HEADER_SIZE);
        assert_eq!(required_region_len(0, 32), HEADER_SIZE);
    }
}
