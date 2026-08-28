//! What a POD topic's ring actually looks like in shared memory.
//!
//! Small POD topics use the **co-located** slot geometry: the readiness stamp
//! and its payload occupy one cache line, so receiving a message costs a single
//! coherence miss.
//!
//! ```text
//! [ HEADER 640 ][ SLOT 0 ][ SLOT 1 ] ...   slot = [ stamp u64 | payload | pad ]
//! ```
//!
//! This file used to pin the opposite. `with_capacity_and_kind` allocated 64
//! bytes per slot and its comment described exactly the layout above, but the
//! stamps were allocated in a separate array between the header and the data,
//! and every POD path addressed the data as `(cached_data_ptr as *T).add(index)`
//! — a stride of `size_of::<T>()`. So the padding was paid for and the
//! co-location never happened: the data region was over-allocated 4x for a
//! 16-byte message, and four consecutive slots shared one cache line, which is
//! the false sharing the 64-byte allocation was evidently meant to prevent.
//!
//! `colo_layout_selected_for_small_pod_types` "verified" that layout by
//! asserting `size_of::<T>() + 8 <= 64` — the branch condition restated, true
//! at compile time, and never once reading shared memory.
//!
//! The stride was changed to match the allocation only after the two
//! implementations that compute it independently were moved with it:
//! `communication::write_topic_slot_bytes` (the writer `horus_net`'s
//! `ShmRingWriter` drives, fed by unauthenticated network data) and
//! `topic::header::{read_slot_inner, slot_stamp}`. Both now read `layout_kind`
//! out of the region rather than re-deriving it, because a reader that
//! disagreed with the writer about where a stamp lives would read payload bytes
//! as a stamp.
//!
//! These tests read the backing file byte for byte, so they fail if any one of
//! those pieces moves alone. `a_large_pod_type_stays_on_the_split_layout` is
//! the control: without it, a build that applied colo unconditionally would
//! satisfy every other test here.
//!
//! Run: `cargo test -p horus_core --test pod_ring_slot_geometry -- --test-threads=1`

use std::mem::size_of;

use horus_core::communication::topic::shm_layout;
use horus_core::communication::Topic;
use horus_core::memory::shm_topics_dir;

// 16 bytes, no Drop — POD by `communication::pod::is_pod`, and comfortably
// inside the `type_size + 8 <= 64` bound that selects the colo geometry.
horus_core::message! {
    #[fixed]
    GeomProbe {
        tag: u64,
        echo: u64,
    }
}

// 72 bytes: past COLO_MAX_PAYLOAD (56), so this one must stay on the split
// layout. It is what stops the rest of this file passing under a build that
// co-located everything unconditionally.
horus_core::message! {
    #[fixed]
    BigProbe {
        tag: u64,
        a: u64,
        b: u64,
        c: u64,
        d: u64,
        e: u64,
        f: u64,
        g: u64,
        h: u64,
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
// The declared geometry
// ---------------------------------------------------------------------------

/// The header declares the colo layout, and a 64-byte slot for a 16-byte POD.
#[test]
fn header_declares_the_colo_layout_for_a_small_pod_type() {
    assert!(
        size_of::<GeomProbe>() <= shm_layout::COLO_MAX_PAYLOAD,
        "this test only means anything for a colo-eligible type"
    );

    let name = unique("hdr");
    let (_tx, region) = filled_ring(&name);

    assert_eq!(
        region[shm_layout::OFF_LAYOUT_KIND],
        shm_layout::LAYOUT_COLO,
        "a small POD topic must record the colo layout in its header"
    );
    assert_eq!(
        u32_at(&region, shm_layout::OFF_TYPE_SIZE) as usize,
        size_of::<GeomProbe>(),
        "header type_size must be the Rust type's size"
    );
    assert_eq!(
        u32_at(&region, shm_layout::OFF_SLOT_SIZE) as usize,
        CACHE_LINE,
        "slot_size is the colo stride: stamp + payload rounded to a line"
    );
    assert_eq!(
        u32_at(&region, shm_layout::OFF_CAPACITY) as usize,
        CAPACITY as usize,
        "capacity is already a power of two"
    );
}

/// A POD type too large to share a line with its stamp keeps the split layout.
///
/// The control case. Every other test here would also pass if colo were applied
/// to every POD type regardless of size, which would put a 72-byte payload
/// across two lines and buy nothing.
#[test]
fn a_large_pod_type_stays_on_the_split_layout() {
    assert!(
        size_of::<BigProbe>() > shm_layout::COLO_MAX_PAYLOAD,
        "BigProbe must be past the colo bound for this to be a control"
    );

    let name = unique("big");
    let tx: Topic<BigProbe> =
        Topic::with_capacity(&name, CAPACITY, None).expect("create big POD topic");
    tx.send(BigProbe {
        tag: tag_of(0),
        a: 0,
        b: 0,
        c: 0,
        d: 0,
        e: 0,
        f: 0,
        g: 0,
        h: 0,
    });
    let region = read_region(&name);

    assert_eq!(
        region[shm_layout::OFF_LAYOUT_KIND],
        shm_layout::LAYOUT_SPLIT,
        "a {}-byte payload cannot share a 64-byte line with an 8-byte stamp",
        size_of::<BigProbe>()
    );
    // And it really is the split geometry: the payload sits past the stamp
    // array, not eight bytes past the header.
    let capacity = u32_at(&region, shm_layout::OFF_CAPACITY) as usize;
    let type_size = u32_at(&region, shm_layout::OFF_TYPE_SIZE) as usize;
    assert_eq!(
        u64_at(&region, shm_layout::data_slot_offset(capacity, 0, type_size)),
        tag_of(0),
        "split payload must live at the split offset"
    );
}

// ---------------------------------------------------------------------------
// The geometry the writer actually uses
// ---------------------------------------------------------------------------

/// Every payload lands at its colo slot, and nowhere else.
///
/// The tagged scan walks the whole ring because slot 0 aliases under either
/// geometry — a single-message test cannot tell them apart, which is how the
/// original version of this check came to be worthless.
#[test]
fn every_payload_lands_at_its_colo_slot() {
    let name = unique("stride");
    let (_tx, region) = filled_ring(&name);

    let capacity = u32_at(&region, shm_layout::OFF_CAPACITY) as usize;
    let type_size = u32_at(&region, shm_layout::OFF_TYPE_SIZE) as usize;
    let slot_size = u32_at(&region, shm_layout::OFF_SLOT_SIZE) as usize;

    for i in 0..capacity {
        assert_eq!(
            u64_at(&region, shm_layout::colo_payload_offset(i, slot_size)),
            tag_of(i as u64),
            "message {i} must sit at its colo payload offset"
        );
    }

    // The old split geometry must NOT also hold, or this test cannot fail.
    let split_data = shm_layout::data_region_offset(capacity);
    let split_holds =
        (0..capacity).all(|i| u64_at(&region, split_data + i * type_size) == tag_of(i as u64));
    assert!(
        !split_holds,
        "payloads still satisfy the split geometry — the scan is not discriminating"
    );
}

/// The stamp and its payload occupy the same cache line.
///
/// This is the entire claim of the layout. It is what turns a receive from two
/// coherence misses into one.
#[test]
fn the_stamp_shares_a_cache_line_with_its_payload() {
    let name = unique("colo");
    let (_tx, region) = filled_ring(&name);

    let capacity = u32_at(&region, shm_layout::OFF_CAPACITY) as usize;
    let slot_size = u32_at(&region, shm_layout::OFF_SLOT_SIZE) as usize;

    for i in 0..capacity {
        let stamp_off = shm_layout::colo_stamp_offset(i, slot_size);
        let payload_off = shm_layout::colo_payload_offset(i, slot_size);
        assert_eq!(
            stamp_off / CACHE_LINE,
            payload_off / CACHE_LINE,
            "slot {i}: stamp at {stamp_off} and payload at {payload_off} must \
             share one cache line"
        );
        // The stamp is real, not incidentally-zero memory: the producer
        // published `sequence + 1` there, with SLOT_WRITING cleared.
        assert_eq!(
            u64_at(&region, stamp_off),
            i as u64 + 1,
            "slot {i} must carry its published stamp"
        );
    }
}

/// No two slots share a cache line.
///
/// The split layout strided payloads by `size_of::<T>()` while allocating 64
/// bytes each, so four 16-byte slots landed in one line and a producer writing
/// slot `i` invalidated the line a consumer was reading slot `i-1` from, three
/// times in four.
#[test]
fn no_two_slots_share_a_cache_line() {
    let name = unique("line");
    let (_tx, region) = filled_ring(&name);

    let capacity = u32_at(&region, shm_layout::OFF_CAPACITY) as usize;
    let slot_size = u32_at(&region, shm_layout::OFF_SLOT_SIZE) as usize;

    assert_eq!(
        shm_layout::HEADER_SIZE % CACHE_LINE,
        0,
        "slots start on a cache-line boundary"
    );
    assert_eq!(slot_size % CACHE_LINE, 0, "a slot is whole cache lines");

    let mut lines: Vec<usize> = (0..capacity)
        .map(|i| shm_layout::colo_slot_offset(i, slot_size) / CACHE_LINE)
        .collect();
    let before = lines.len();
    lines.sort_unstable();
    lines.dedup();
    assert_eq!(
        lines.len(),
        before,
        "every slot must own its cache line, so no two may collide"
    );
}

/// Colo has no sequence array, and no over-allocation.
#[test]
fn colo_has_no_sequence_array_and_no_over_allocation() {
    let name = unique("bounds");
    let (_tx, region) = filled_ring(&name);

    let capacity = u32_at(&region, shm_layout::OFF_CAPACITY) as usize;
    let slot_size = u32_at(&region, shm_layout::OFF_SLOT_SIZE) as usize;

    // Slot 0's stamp begins immediately after the header. Under the split
    // layout this offset held the first entry of a `capacity * 8` stamp array
    // and the data began `capacity * 8` bytes later.
    assert_eq!(
        shm_layout::colo_slot_offset(0, slot_size),
        shm_layout::HEADER_SIZE,
        "colo slots start at the header's end — there is no stamp array"
    );

    let required = shm_layout::colo_required_region_len(capacity, slot_size);
    assert!(
        region.len() >= required,
        "region ({} bytes) is smaller than the geometry the header declares \
         ({required} bytes)",
        region.len()
    );
    // The split layout would have needed the stamp array on top of the same
    // slots, so colo is strictly smaller for the identical ring.
    let split_equivalent = required + capacity * core::mem::size_of::<u64>();
    assert!(
        required < split_equivalent,
        "colo must not be larger than the split layout it replaces"
    );
}

// ---------------------------------------------------------------------------
// Reader and writer agree
// ---------------------------------------------------------------------------

/// A consumer reads back exactly what the producer wrote, in order.
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
    while let Some(v) = rx.recv() {
        got.push(v);
    }
    assert_eq!(
        got.len(),
        CAPACITY as usize,
        "every published message must be readable"
    );
    for (i, v) in got.iter().enumerate() {
        assert_eq!(v.tag, tag_of(i as u64), "message {i} tag");
        assert_eq!(v.echo, !tag_of(i as u64), "message {i} echo");
    }
}
