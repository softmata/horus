//! What a POD topic's ring actually looks like in shared memory.
//!
//! `RingTopic::with_capacity_and_kind` allocates **64 bytes per slot** for any
//! POD type with `size_of::<T>() + 8 <= 64`, and its comment describes a
//! co-located `[seq(8) | data(T) | pad to 64]` slot. No POD path implements
//! that layout. Every POD writer and every POD reader addresses the data region
//! as `(cached_data_ptr as *T).add(index)` — a stride of `size_of::<T>()` — and
//! the sequence numbers live in their own array between the header and the data
//! (`shm_layout::seq_slot_offset`), not inside the slots.
//!
//! Writers and readers therefore agree with each other, so this is **not**
//! corruption. What it is:
//!
//!   * the data region is over-allocated by `64 / size_of::<T>()` — 4x for a
//!     16-byte message, 8x for an 8-byte one; and
//!   * consecutive slots are `size_of::<T>()` apart, so four 16-byte slots
//!     share one 64-byte cache line and a producer writing slot `i` invalidates
//!     the line a consumer is reading slot `i-1` from, three times in four —
//!     the false sharing the 64-byte allocation was evidently meant to prevent.
//!
//! The same comment claims the 64-byte geometry is part of the Python↔Rust wire
//! contract ("horus_py computes slot size the same way"). `horus_py` is a PyO3
//! wrapper: `Topic(CmdVel)` calls `horus_core::Topic::<CmdVel>::with_capacity`,
//! the very code under test here, so there is no second implementation to
//! disagree with. The implementations that *do* compute this stride
//! independently are in Rust — `horus_net::ShmRingWriter` and
//! `topic::header::read_latest_slot_bytes` — and both use `type_size` for POD,
//! matching the dispatch paths rather than the 64. Any future change to the POD
//! stride has to move those two with it.
//!
//! These tests pin that truth so a one-sided change cannot land quietly.
//!
//! Run: `cargo test -p horus_core --test pod_ring_slot_geometry -- --test-threads=1`

use std::mem::size_of;

use horus_core::communication::topic::shm_layout;
use horus_core::communication::Topic;
use horus_core::memory::shm_topics_dir;

// 16 bytes, no Drop — POD by `communication::pod::is_pod`, and comfortably
// inside the `type_size + 8 <= 64` branch that allocates 64-byte slots.
horus_core::message! {
    #[fixed]
    GeomProbe {
        tag: u64,
        echo: u64,
    }
}

const CAPACITY: u32 = 8;
const CACHE_LINE: usize = 64;

fn unique(tag: &str) -> String {
    use std::sync::atomic::{AtomicU32, Ordering};
    static N: AtomicU32 = AtomicU32::new(0);
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.subsec_nanos())
        .unwrap_or(0);
    format!(
        "podgeom_{tag}_{}_{}_{}",
        std::process::id(),
        nanos,
        N.fetch_add(1, Ordering::Relaxed)
    )
}

/// A distinct, non-zero marker for message `i`. Distinct per index is what lets
/// the raw-byte scan below tell a 16-byte stride from a 64-byte one.
fn tag_of(i: u64) -> u64 {
    0xC0FF_EE00_0000_0000 | (i + 1)
}

fn probe(i: u64) -> GeomProbe {
    GeomProbe {
        tag: tag_of(i),
        echo: !tag_of(i),
    }
}

/// Read the topic's backing file. The mapping is `MAP_SHARED` and file-backed,
/// so the bytes a `send` wrote are visible here.
fn read_region(name: &str) -> Vec<u8> {
    let path = shm_topics_dir().join(name);
    std::fs::read(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

fn u32_at(bytes: &[u8], off: usize) -> u32 {
    let raw: [u8; 4] = bytes[off..off + 4].try_into().expect("4 bytes in region");
    u32::from_ne_bytes(raw)
}

fn u64_at(bytes: &[u8], off: usize) -> u64 {
    let raw: [u8; 8] = bytes[off..off + 8].try_into().expect("8 bytes in region");
    u64::from_ne_bytes(raw)
}

/// Fill a fresh POD ring with `CAPACITY` tagged messages and hand back the raw
/// region. The `Topic` is returned alongside so the caller keeps the SHM file
/// alive (dropping the owning handle unlinks it).
fn filled_ring(name: &str) -> (Topic<GeomProbe>, Vec<u8>) {
    let tx: Topic<GeomProbe> =
        Topic::with_capacity(name, CAPACITY, None).expect("create POD topic");
    for i in 0..CAPACITY as u64 {
        tx.send(probe(i));
    }
    let region = read_region(name);
    (tx, region)
}

// ---------------------------------------------------------------------------
// The allocated geometry
// ---------------------------------------------------------------------------

/// The header advertises 64-byte slots for a 16-byte POD message.
#[test]
fn header_advertises_sixty_four_byte_slots_for_a_small_pod_type() {
    assert!(
        size_of::<GeomProbe>() + 8 <= CACHE_LINE,
        "this test only means anything inside the `type_size + 8 <= 64` branch"
    );

    let name = unique("hdr");
    let (_tx, region) = filled_ring(&name);

    assert_eq!(
        u32_at(&region, shm_layout::OFF_TYPE_SIZE) as usize,
        size_of::<GeomProbe>(),
        "header type_size must be the Rust type's size"
    );
    assert_eq!(
        u32_at(&region, shm_layout::OFF_SLOT_SIZE) as usize,
        CACHE_LINE,
        "with_capacity_and_kind pins slot_size to 64 for a small POD type"
    );
    assert_eq!(
        u32_at(&region, shm_layout::OFF_CAPACITY) as usize,
        CAPACITY as usize,
        "capacity is already a power of two"
    );
}

// ---------------------------------------------------------------------------
// The geometry the writer actually uses
// ---------------------------------------------------------------------------

/// The POD send path strides by `size_of::<T>()`, not by the header's 64.
///
/// If this ever fails because the payloads land 64 bytes apart, the stride has
/// been changed to match the allocation — which is the intended optimize-phase
/// fix, but only lands correctly if `horus_net::ShmRingWriter` (`stride =
/// type_size` for POD) and `topic::header::read_latest_slot_bytes` (same) moved
/// with it. Update this test *after* checking those two, not before.
#[test]
fn pod_writer_strides_by_size_of_t_not_by_the_header_slot_size() {
    let name = unique("stride");
    let (_tx, region) = filled_ring(&name);

    let capacity = u32_at(&region, shm_layout::OFF_CAPACITY) as usize;
    let type_size = u32_at(&region, shm_layout::OFF_TYPE_SIZE) as usize;
    let slot_size = u32_at(&region, shm_layout::OFF_SLOT_SIZE) as usize;
    let data = shm_layout::data_region_offset(capacity);

    // Does message `i` sit at `data + i * stride` for every i?
    let holds = |stride: usize| -> bool {
        (0..capacity).all(|i| u64_at(&region, data + i * stride) == tag_of(i as u64))
    };

    assert!(
        holds(type_size),
        "every POD dispatch path indexes `(cached_data_ptr as *T).add(index)`, \
         so the payloads must be {type_size} bytes apart"
    );
    assert!(
        !holds(slot_size),
        "payloads are NOT {slot_size} bytes apart — the header's slot_size is \
         the allocation stride only, never the index stride"
    );

    // Slot 0 is where the two geometries agree, and it is the only place they
    // do. A single-message test cannot tell them apart; that is why the tagged
    // scan above walks the whole ring.
    assert_eq!(
        u64_at(&region, data),
        tag_of(0),
        "slot 0 aliases under either stride"
    );
    assert_ne!(
        type_size, slot_size,
        "the allocation stride and the index stride currently disagree"
    );
}

/// The disagreement is pure over-allocation, never an out-of-bounds write.
///
/// This is the invariant that must hold whichever way the geometry is settled:
/// the region has to be at least as large as `capacity * index_stride`, and the
/// allocation stride has to be at least the index stride.
#[test]
fn the_over_allocation_is_in_bounds_and_costs_four_x() {
    let name = unique("bounds");
    let (_tx, region) = filled_ring(&name);

    let capacity = u32_at(&region, shm_layout::OFF_CAPACITY) as usize;
    let type_size = u32_at(&region, shm_layout::OFF_TYPE_SIZE) as usize;
    let slot_size = u32_at(&region, shm_layout::OFF_SLOT_SIZE) as usize;
    let data = shm_layout::data_region_offset(capacity);

    assert!(
        slot_size >= type_size,
        "allocation stride {slot_size} is smaller than the index stride \
         {type_size} — that would be an out-of-bounds ring write"
    );
    assert!(
        region.len() >= data + capacity * slot_size,
        "region ({} bytes) is smaller than the geometry the header declares \
         ({} bytes)",
        region.len(),
        data + capacity * slot_size
    );

    let used = capacity * type_size;
    let allocated = capacity * slot_size;
    assert_eq!(
        allocated / used,
        4,
        "a 16-byte POD message gets 64 bytes of ring per slot: {allocated} \
         bytes allocated for {used} bytes of messages"
    );
}

/// Four consecutive POD slots land in one cache line.
///
/// This is the false-sharing cost of the stride/allocation mismatch: the
/// 64-byte allocation was meant to give each slot its own line, and the
/// `size_of::<T>()` stride puts four of them back on the same one.
#[test]
fn four_consecutive_pod_slots_share_one_cache_line() {
    let name = unique("line");
    let (_tx, region) = filled_ring(&name);

    let capacity = u32_at(&region, shm_layout::OFF_CAPACITY) as usize;
    let type_size = u32_at(&region, shm_layout::OFF_TYPE_SIZE) as usize;
    let data = shm_layout::data_region_offset(capacity);

    assert_eq!(
        data % CACHE_LINE,
        0,
        "the data region starts on a cache-line boundary ({data})"
    );

    let line_of = |i: usize| (data + i * type_size) / CACHE_LINE;
    assert_eq!(
        line_of(0),
        line_of(3),
        "slots 0..=3 are {type_size} bytes apart, so all four are in one line"
    );
    assert_ne!(line_of(0), line_of(4), "slot 4 starts the next line");

    // Per-slot sequence numbers live in their own array, not co-located with
    // the data the way `with_capacity_and_kind`'s comment describes.
    assert!(
        shm_layout::seq_slot_offset(capacity - 1) < data,
        "the whole sequence array precedes the data region"
    );
    assert_eq!(
        shm_layout::seq_slot_offset(1) - shm_layout::seq_slot_offset(0),
        8,
        "sequence stamps are a bare u64 array, not a per-slot header"
    );
}

// ---------------------------------------------------------------------------
// Reader and writer agree — so this is waste, not corruption
// ---------------------------------------------------------------------------

/// A consumer reads back exactly what the producer wrote, in order.
///
/// The POD recv paths use the same `(cached_data_ptr as *T).add(index)` stride
/// as the send paths, so the mismatch with the 64-byte allocation costs memory
/// and cache lines but never a byte of data.
#[test]
fn pod_reader_and_writer_agree_on_the_stride() {
    let name = unique("agree");
    let tx: Topic<GeomProbe> =
        Topic::with_capacity(&name, CAPACITY, None).expect("create producer handle");
    let rx: Topic<GeomProbe> =
        Topic::with_capacity(&name, CAPACITY, None).expect("create consumer handle");

    // Register the consumer before anything is published: a broadcast-backend
    // consumer seats its cursor at the head when it joins, so a handle that
    // first calls recv() after the sends would legitimately see nothing.
    assert!(rx.recv().is_none(), "nothing published yet");

    for i in 0..CAPACITY as u64 {
        tx.send(probe(i));
    }

    let mut got = Vec::new();
    for _ in 0..(CAPACITY as usize * 8) {
        match rx.recv() {
            Some(m) => got.push(m),
            None if got.len() == CAPACITY as usize => break,
            None => std::thread::sleep(std::time::Duration::from_millis(1)),
        }
    }

    assert_eq!(
        got.len(),
        CAPACITY as usize,
        "expected every published message back, got {}",
        got.len()
    );
    for (i, m) in got.iter().enumerate() {
        let i = i as u64;
        assert_eq!(m.tag, tag_of(i), "message {i} tag");
        assert_eq!(m.echo, !tag_of(i), "message {i} echo");
    }
}
