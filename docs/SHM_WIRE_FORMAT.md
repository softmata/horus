# Changing the shared-memory topic wire format

Every topic is a single mmap'd region shared between unrelated processes. Its
layout is a wire format in the strict sense: two binaries built from different
commits address the same bytes at the same time, and neither has a way to
renegotiate. There is no framing, no length-delimited field, no schema on the
wire. A reader that disagrees with a writer about where a byte lives does not
fail — it returns the wrong value, or writes over a control word, and keeps
running.

That has already happened here. `horus_net` held a copied table of byte offsets
in a comment, the copy went stale, and because the backing-file path in the same
module was also wrong the code never ran, so nothing noticed from 2026-03-29
until the 2026-07-30 security audit (`horus_net/src/shm_writer.rs:10-32`). All
four copied facts were wrong by then: the data region omitted the sequence
array, the `messages_total` offset actually pointed at `topic_kind`, the serde
length word was written where the ready flag lives, and the per-slot ready flag
was never published at all
(`horus_core/src/communication/topic/shm_layout.rs:11-16`). The writer in
question is fed by unauthenticated network data, so each of those is
remote-controlled corruption of a live IPC control structure.

`shm_layout` exists to make that class of drift a build failure rather than a
four-month data-corruption bug. This document is the procedure around it: what
still has to be done by hand, and what nothing checks.

## Scope

**Covered.** The topic region: `TopicHeader`, the per-slot sequence array, the
two POD slot geometries, and the serde slot shape. The types and constants live
in `horus_core/src/communication/topic/header.rs` and
`horus_core/src/communication/topic/shm_layout.rs`.

**Not covered, and each has its own rules:**

| Format | Where | How it is versioned |
|---|---|---|
| `horus_net`'s UDP packet header | `horus_net/src/wire.rs` | Its own `MAGIC` (`:20`) and `VERSION` (`:23`, currently 1), both checked in `PacketHeader::decode` (`:74`, `:78`) |
| The C++ message-struct layout contract | `horus_cpp/src/layout_contract.rs` | A generated header emitting `static_assert(offsetof(...))` per field (`:149`) plus a `sizeof` assert (`:142`); regenerate with the command at `horus_cpp/src/layout_contract.rs:37-39` |
| The global log buffer | `horus_core/src/core/log_buffer.rs:88` | Its own private `HEADER_SIZE = 64` |
| The `FanoutShm` channel matrix | `horus_core/src/communication/topic/shm_fanout.rs` | See below — it shares the region but not the version field |

The fanout case is the one that catches people. `FanoutShm` reuses the *same*
topic SHM file, placing `FanoutShmMeta` at offset 4096, after `TopicHeader` and
its page padding (`shm_fanout.rs:12-30`, `:99`). Its layout version is the
**top** byte of `FANOUT_MAGIC`, `0x0300_5455_4F4E_4146` (`shm_fanout.rs:96`) —
the eighth byte of the `"FANOUT\0"` tag as it sits on the wire, currently 3, not
the low byte. A change to the channel layout bumps that byte, not
`TOPIC_VERSION`: `attach` returns `None` for any non-zero magic that is not the
current one (`shm_fanout.rs:589-594`), and the caller rebuilds on `SpscShm`
rather than reading the old region with the new strides (`shm_fanout.rs:87-95`,
`:576-579`). If your change is entirely inside the fanout matrix, this document
does not apply to it.

## The region, as it actually is

`TopicHeader` is 640 bytes, ten cache lines, `#[repr(C, align(64))]`
(`header.rs:232-233`), pinned by `const _: () = assert!(mem::size_of::<TopicHeader>() == 640);`
at `header.rs:434`. After it, one of two geometries:

```text
split (LAYOUT_SPLIT = 0)
[ HEADER 640 ][ SEQ_ARRAY capacity*8 ][ DATA capacity*stride ]

colo (LAYOUT_COLO = 1)
[ HEADER 640 ][ SLOT 0 ][ SLOT 1 ] …   slot = [ stamp u64 | payload | pad to 64 ]
```

In the split layout `stride` is `type_size` for POD topics and `slot_size` for
serde topics (`shm_layout.rs:26-36`). A serde slot is
`[ 8B ready | 8B length | data… ]` (`shm_layout.rs:91-97`).

The geometry is chosen once, at creation, by
`colo_eligible(is_pod, type_size)` — POD, non-zero, and at most
`COLO_MAX_PAYLOAD` (56, being `CACHE_LINE` 64 minus `COLO_PAYLOAD_OFF` 8) bytes
(`shm_layout.rs:193`, `:198`, `:206`, `:213-215`) — and recorded in
`layout_kind` (`header.rs:538-545`). Every later attacher reads that byte rather
than re-deriving it, so a producer and a consumer built with different
eligibility bounds cannot disagree about where a stamp lives
(`header.rs:493-502`). A colo topic's stored `slot_size` is
`colo_slot_size(type_size)`, not `type_size`
(`mod.rs:1009-1019`, stored at `header.rs:552`).

Readiness lives in two different places, which is the single most common source
of a half-correct writer: for POD topics it is in the sequence array (split) or
the slot's own first word (colo); for serde topics it is the first eight bytes
of the slot. Both store `sequence + 1`, with `SLOT_WRITING` (bit 63) set while a
producer is mid-write (`shm_layout.rs:38-50`, `:99-115`). A writer that
publishes only `sequence_or_head` is visible to `SpscShm` consumers and
invisible to `MpscShm` ones — silent message loss, not a loud failure.

## Does this change need a `TOPIC_VERSION` bump?

`TOPIC_VERSION` is `4` (`header.rs:66`). The history recorded in its doc comment
is `v2` (added `slot_size`), `v3` (added the per-slot sequence array), `v4` (two
breaks at once: every millisecond timestamp moved from `CLOCK_REALTIME` to
`CLOCK_MONOTONIC`, and `layout_kind` was added) — `header.rs:24-65`.

That history is incomplete, and the gap is worth knowing before you trust it as
a model. `shm_layout.rs:14` records that `messages_total` sits at offset 136 and
"was 56 before v4". A field that moves is the loudest class of break there is,
and the v4 entry does not mention it. Write your own entry as though someone
will later reconstruct the wire format from it alone, because that is what this
comment is for.

The test is not "did bytes move". It is **can a process built before this change
misread a region written after it, or the reverse, without noticing**.

| Change | Bump? | Why |
|---|---|---|
| A field moves, grows, or shrinks | **Yes** | Every old reader addresses the wrong bytes. |
| A field's *meaning* changes at the same size and offset | **Yes** | This is v4(a). The timestamps did not move; they changed epoch, and the two epochs are compared through shared memory with nothing to distinguish them (`header.rs:44-52`). |
| A new field carved out of existing padding, that an old reader must consult to address the data correctly | **Yes** | This is v4(b). `layout_kind` displaces nothing, but a v3 reader has no concept of it and would address a colo region with split offsets (`header.rs:33-38`, `:54-55`). |
| A new field in padding that an old reader can ignore and still be correct | No | See `layout_hash` below — it was added exactly this way and did not bump the version. Old readers keep working; new readers must tolerate the field reading 0 on a region an old writer created. |
| The header grows or shrinks in total | **Yes**, and see below | `HEADER_SIZE` is the base of every derived offset. |
| `colo_eligible`'s bound changes | No, but read on | `layout_kind` is stamped at creation and read, never re-derived, so mixed builds stay consistent on any one region. Existing regions keep their geometry. |
| The `FanoutShm` channel layout changes | No | Bump the top byte of `FANOUT_MAGIC` instead (`shm_fanout.rs:87-96`). |
| A field is added to `TopicSlotRead` or another in-process type | No | Not on the wire. |

`TOPIC_MAGIC` is deliberately *not* a version discriminator. It is
`0x4144415054495645` ("ADAPTIVE") and the comment at `header.rs:22` marks it
"kept for backwards compat". Nothing in the tree treats the magic as a version,
and every raw-mmap path checks only it — which is the whole of the runtime
section below.

### The literal form of the constant is load-bearing

Four consumers read `TOPIC_VERSION` by parsing the source line, because the
constant is `pub(crate)` and cannot be imported:

- `horus_manager/src/version.rs:1112-1114` strips the exact prefixes
  `pub(crate) const TOPIC_VERSION: u32 = ` or `pub const TOPIC_VERSION: u32 = `,
  and panics if neither matches (`:1117`).
- `horus_manager/src/commands/doctor.rs:1572-1581` and
  `horus_manager/src/commands/upgrade.rs:721-730` take the first line containing
  `const TOPIC_VERSION` and an `=`, then read after the last `=` (`rsplit('=')`).
- `install.sh:816-818` greps `-m1` for `TOPIC_VERSION: u32` and seds
  `s/.*= *\([0-9][0-9]*\).*/\1/`. The leading `.*` is greedy, so it reads the
  digits after the *last* `=` — which is why the `32` in `u32` is not matched.
- `.github/workflows/distribution.yml:559-561` greps `-m1` for
  `const TOPIC_VERSION` and takes the value after the `=`; the comment at
  `:557-558` notes that taking every number on the line would also match the
  `32` in `u32`.

So: keep it one line, keep the `: u32 = N;` shape, and do not put a doc line
containing the string `TOPIC_VERSION: u32` or `const TOPIC_VERSION` above it —
the `-m1` greps and the two `.find()` scans would take that line instead. (No
line above `:66` contains either string today.) Renaming the constant or
changing its visibility keyword breaks the drift test loudly and the two shell
greps silently.

Four test fixtures also synthesise a `header.rs` in that exact shape —
`install_contract.rs:1253`, `doctor.rs:3321` and `:3458`, `upgrade.rs:1576`.
They emit the shape rather than reading it, so a change to the declaration form
leaves them testing the form that no longer exists.

## Which constants move together

`shm_layout` is the only definition. Eleven header offsets are exposed
(`shm_layout.rs:68-86`, `:189`), plus the serde slot offsets (`:91-97`), the
colo constants (`:178-206`), `SLOT_WRITING` (`:115`), `MAGIC` (`:57`),
`HEADER_SIZE` (`:60`) and `IS_POD_YES` (`:63`). Everything else is derived by
the functions at `:125-264` — all `const fn` except the two `*_checked` forms
(`:168`, `:264`), which cannot be, and which are the ones to use on any geometry
read out of an untrusted header.

| If you change | These move with it |
|---|---|
| The order or size of any header field | Its `OFF_*` constant, and any `OFF_*` numerically after it |
| `TopicHeader`'s total size | `HEADER_SIZE`, `seq_array_offset`, `data_region_offset`, `colo_slot_offset` and every `required_region_len*`; the `HEADER_SIZE == 640` assert at `shm_layout.rs:310` and the `size_of` assert at `header.rs:434`; the test assert at `header.rs:2051`; also the two hand-recomputed sites at `header.rs:1603` and `:1617` (they use `TOPIC_HEADER_SIZE`, so they follow, but they do not use the module's helpers) |
| The serde slot shape | `SERDE_SLOT_READY_OFF`, `SERDE_SLOT_LEN_OFF`, `SERDE_SLOT_DATA_OFF`, `SERDE_SLOT_OVERHEAD`; the unit test `serde_slot_layout_matches_the_dispatch_writer` at `shm_layout.rs:364`; the literal `+ 8` and `+ 16` at `header.rs:1624-1625`; `ShmRingWriter::write_serialized` (`horus_net/src/shm_writer.rs:242-287`); `write_topic_slot_bytes` (`horus_core/src/communication/mod.rs:306-311`) |
| `CACHE_LINE`, `COLO_PAYLOAD_OFF` or `COLO_MAX_PAYLOAD` | The `COLO_PAYLOAD_OFF + COLO_MAX_PAYLOAD == CACHE_LINE` assert at `shm_layout.rs:317`, `colo_slot_size`, `colo_eligible` |
| `verbose`'s position | `TOPIC_VERBOSE_OFFSET` at `header.rs:1247` — which is not in `shm_layout` and has **no** compile-time assert; see below |

## What the compile-time asserts do and do not cover

`shm_layout::static_asserts` (`shm_layout.rs:276-323`) is a `const` block, so it
is evaluated by any build of `horus_core`. Nothing needs to run.

**It covers**, by `offset_of!` against the real field (`shm_layout.rs:281-291`):

`magic`, `type_size`, `is_pod`, `topic_kind`, `layout_kind`, `messages_total`,
`migration_epoch`, `sequence_or_head`, `capacity`, `capacity_mask`, `slot_size`
— eleven fields. Reordering or resizing any of them stops the crate compiling.

It additionally pins four structural invariants (`shm_layout.rs:297-321`):

- `topic_kind` is strictly before `messages_total` — the specific mistake
  `horus_net` made was a 64-bit store at offset 48, believing it held the
  counter.
- `messages_total` shares a cache line with neither `migration_epoch` nor
  `sequence_or_head`. This is a performance invariant with a measured number
  behind it: at its old offset it false-shared with the epoch every consumer
  Acquire-loads per `recv`, costing about 36 ns of a ~150 ns one-way latency on
  an i7-10750H (`shm_layout.rs:299-306`, `header.rs:271-280`).
- `HEADER_SIZE == 640`, `OFF_SEQUENCE_OR_HEAD == 64`, and `HEADER_SIZE` is a
  whole number of cache lines.
- `layout_kind` sits between `topic_kind` and `messages_total`, which is what
  makes "carved out of padding, displaces nothing" a checked claim rather than
  a comment.

**It does not cover:**

| Not asserted | What guards it instead |
|---|---|
| `verbose` at offset 23 (`TOPIC_VERBOSE_OFFSET`, `header.rs:1247`) | Two runtime tests, deliberately duplicated: `verbose_offset_constant_correct` (`horus_core/src/communication/topic/tests.rs:10766-10782`, which asserts both the literal and `offset_of!`) and `verbose_offset_matches_struct_layout` (`header.rs:2092-2101`). This is the one wire offset the `const` assertions miss, and `tests.rs:10753-10765` says so explicitly. |
| `version` at offset 8 | Nothing. There is no `OFF_VERSION` anywhere in the workspace. Inside `horus_core` it is reached through `&TopicHeader` (`header.rs:236-238`) so it cannot drift; an out-of-crate reader that wanted to check it would have to hardcode 8. |
| `type_name` at offset 216 | `offset_of!` at both read sites (`header.rs:1657`, `:1871`); no exported constant. |
| `layout_hash` at offset 248, `tail`, `publisher_count`, `subscriber_count`, `backend_mode`, `migration_lock`, `creator_pid`, `type_align`, `stall_tail`, `stall_since_ms`, `last_reap_ms`, the lease timestamps, the sixteen `ParticipantEntry` slots — and every other field not in the eleven above | Nothing outside the crate. `shm_layout` exposes no offset for any of them. For the timestamps and participant entries that is a deliberate boundary (`header.rs:1947-1955`): a monotonic lease value is meaningless off this host, so it must never be replicated. (The same passage says the offsets "stop at `slot_size`", which is loose — `OFF_MESSAGES_TOTAL` is 136, past `OFF_SLOT_SIZE`'s 80. The substantive half, that no timestamp and no participant field is exposed, is true.) |
| That a reader *consults* a field it needs | Nothing. The asserts prove where `layout_kind` is; they cannot prove that a writer reads it. This is a live gap — see the last section. |
| That an out-of-crate copy of the numbers exists at all | Nothing. The asserts only bind constants that are *used*. A file that hardcodes `640` is invisible to them. |

## How a mismatched reader is detected at runtime, and where it is not

This is the part to read twice, because the header's own v4 note
(`header.rs:60-65`) is scoped more narrowly than it first reads.

**Detected.** Attaching a typed `Topic<T>` to an already-initialised region
compares `header.version` against `TOPIC_VERSION` and refuses on mismatch,
naming both numbers — `negotiate_shm_header`,
`horus_core/src/communication/topic/mod.rs:1154-1162`. This is the only place in
the workspace that reads a topic header's `version` field at runtime. Four
further checks sit beside it, and each catches something the version number does
not:

- The stored type name is compared, truncated-as-stored, which catches a type
  mismatch and not a version one (`mod.rs:1164-1210`).
- `type_size` must match for any POD topic, with no `GenericMessage` exemption,
  because the POD data path's stride *is* `size_of::<T>()`
  (`mod.rs:1212-1237`).
- `validate_ring_geometry` re-checks the `capacity`, `capacity_mask` and
  `slot_size` another process stored against the mapping this one holds
  (`mod.rs:1239`, defined at `:1276` with its rationale from `:1242`). Every one
  of those three is attacker-controlled on a shared `/dev/shm`.
- `layout_hash` (offset 248) catches two builds of the *same* message that
  reordered a field: `bind_layout_hash` installs it with a compare-exchange from
  0 and rejects a contradicting value (`mod.rs:878-912`). It is opt-in —
  `Topic::new_checked` supplies it (`mod.rs:4244-4248`), `message!` generates a
  `LAYOUT_HASH` and a `topic()` helper that calls it (`macros.rs:222`, `:246`),
  and `Topic::new` passes nothing. A hash of 0 disables the check, so an older
  peer is never rejected — it is simply not protected (`header.rs:399-404`).
  This field was added without a `TOPIC_VERSION` bump, and that was correct: an
  old reader that ignores it is still addressing every byte correctly.

**Not detected.** Every raw-mmap path validates `TOPIC_MAGIC` and stops there.
Since the magic is unchanged across v2, v3 and v4 by design (`header.rs:22`), it
cannot discriminate. The sites:

| Path | Magic check | Version check |
|---|---|---|
| `read_slot_inner`, the shared body of `read_latest_slot_bytes` and `read_slots_since` (`header.rs:1440`; `read_slots_since` at `:4344`) | yes | no |
| `read_topic_sequence` (`header.rs:1749`) | yes | no |
| `read_topic_messages_total` (`header.rs:1780`) | yes | no |
| `read_topic_header_info` (`header.rs:1849`) | yes | no |
| `write_topic_slot_bytes` (`horus_core/src/communication/mod.rs:192`) | yes | no |
| `ShmRingWriter::open_path` (`horus_net/src/shm_writer.rs:79-82`) | yes | no |
| `SubscriptionFreshness::refresh` (`horus_core/src/scheduling/types.rs:692`, magic at `:710`) | yes | no |

So the accurate statement is: **a v4 binary refuses to open a v3 segment as a
typed `Topic<T>`, and reads or writes it happily through every other path.**
`horus topic echo`, `horus topic list`, `horus topic hz`, `horus topic bw` and
`horus_net`'s importer would all proceed. Whether that matters depends entirely
on whether the change moved a byte those paths touch — which is why the bump
decision above is about misreading, not about tidiness.

**Detected out of band, weakly.** `horus_manager` keeps a hand-copied mirror,
`CLI_TOPIC_VERSION = 4` (`horus_manager/src/version.rs:24`), and compares it
against the `topic_version` field that `install.sh:816-830` records in
`~/.horus/install_manifest.toml` (or `$HORUS_PREFIX` when set —
`version.rs:63-76`). A difference is reported as an ABI break
(`version.rs:248-251`) but only *warns* and continues (`version.rs:365-371`); it
becomes a hard failure only when `HORUS_STRICT_VERSION` is set to `1`, `true` or
`yes` (`version.rs:312-321`, `:346-357`). And it compares what the installer
wrote down, not what any live segment says. It exists because of a real
incident: at RC1 the binary came from a tag and the cached source from `main`,
both calling themselves 0.4.0, 93 commits apart, with `TOPIC_VERSION` 3 at the
tag and 4 on main (`horus_manager/tests/install_contract.rs:1749-1755`).

## Also update these

Grepping the workspace for `TOPIC_VERSION` and for the `OFF_*` constants gives
the following. The point of the table is that the first group is the group that
bites: those files carry the knowledge, and only some of them are bound to the
source of truth by anything.

### Carries the layout or the number

| File | What it holds | Bound to the source by |
|---|---|---|
| `horus_net/src/shm_writer.rs` | The only user of the `OFF_*` constants outside the `horus_core` package. Imports `shm_layout as layout` (`:39`) and writes through `data_slot_offset`, `seq_slot_offset`, `SERDE_SLOT_*` and `SLOT_WRITING`. | The compiler — it uses the constants rather than copies. An offset change follows automatically; a *new* field it must consult does not. |
| `horus_manager/src/version.rs:24` | `CLI_TOPIC_VERSION`, a hand-copied `4`. | `cli_topic_version_matches_horus_core` (`version.rs:1096-1124`), which parses `header.rs` by literal prefix. |
| `horus_manager/src/commands/doctor.rs:1562-1582` | `topic_version_in`, greps a *different* checkout's `header.rs`. | Nothing but the line's shape. |
| `horus_manager/src/commands/upgrade.rs:711-731` | `topic_version_of`, a private second copy of the same grep. | Nothing but the line's shape. |
| `install.sh:816-830` | Writes `topic_version` into `install_manifest.toml`. | `install_contract.rs:1673-1747` asserts the manifest names it. |
| `horus_manager/tests/install_contract.rs:1253` | A test rig that *emits* a synthetic `TOPIC_VERSION` line. | Nothing. It reproduces the declaration shape rather than reading it. |
| `.github/workflows/distribution.yml:554-564` | Cross-checks the manifest's `topic_version` against the cached tree's `header.rs`. | Runs on every PR to `main`/`dev`, every push to `main`, and daily (`distribution.yml:28-40`; the `tag-coherence` job at `:452` carries no `if:`) — but it installs and inspects the **published release**, so it cannot see the `TOPIC_VERSION` in your branch. |
| `horus_core/fuzz/fuzz_targets/fuzz_shm_header.rs` | A hardcoded re-implementation: `TOPIC_MAGIC` at `:22`, `HEADER_SIZE = 640` at `:24`, field offsets `12/20/64/72/76/80` at `:42-47`. | **Nothing.** See below. |
| `horus-docs` `content/docs/getting-started/installation.mdx:891` | A sample manifest printing `topic_version = 4`. | Nothing. No script under `horus-docs/scripts/` mentions `topic_version` or `TOPIC_VERSION`. |
| `horus-docs` `content/docs/troubleshooting.mdx:426-427` | Sample output showing `CLI topic ABI: 4` against an installed 3. | Nothing. Illustrative of a mismatch rather than of the current value, but the `4` is still a copy. |
| `horus-docs` `content/docs/development/cli-reference.mdx:1091` | Prose describing the doctor cross-check. | Nothing. |

### Reads the region only through `horus_core`'s API — no edit needed

These are the files the original bug report would have you expect in the table,
and they are not in it, which is the good news. They go through
`read_topic_header_info`, `read_topic_messages_total`, `read_latest_slot_bytes`
and `read_slots_since`, all of which derive their offsets from `offset_of!`:

- `horus_net/src/shm_reader.rs:21` — the export reader.
- `horus_manager/src/discovery/topics.rs:4`, `:105` — topic discovery and the
  rate sampler. The comment at `:99-102` records that this file used to quote
  byte offsets in prose and that the numbers had gone stale; they were removed
  rather than corrected.
- `horus_manager/src/commands/topic.rs` — `echo_topic` via `read_slots_since`
  (`:161`, `:269`), `topic_hz` via `read_topic_messages_total` (`:884-885`),
  `topic_bw` via `read_latest_slot_bytes` (`:1024-1025`, `:1090`).

`horus_manager` never mmaps a topic region itself. It declares `memmap2`
(`horus_manager/Cargo.toml:96`) and never uses it: the only `mmap` mentions in
`horus_manager/src` are a comment in `commands/log.rs:353`, a seccomp
allow-list entry in `plugins/sandbox.rs:29`, and a crate name in
`source_resolver.rs:492`.

`horus_py` and `horus_cpp` hold no topic-header layout knowledge at all — the
only `640`s in either are image dimensions, and `horus_cpp`'s layout contract is
about message structs, a different format (`mod.rs:1004-1008` says the same). If
you add a direct mapper to any of them, it belongs in the first table and it
must use `shm_layout`.

### The exported byte-23 toggle

`TOPIC_VERBOSE_OFFSET` and `set_topic_verbose` are re-exported from the crate
root behind `#[doc(hidden)]` (`horus_core/src/lib.rs:62-65`, and again at
`communication/mod.rs:39` and `topic/mod.rs:544-545`) and documented as being
for "external tools like the TUI monitor" (`header.rs:1240-1247`). Grepping the
workspace for either name, **no caller exists outside `horus_core`'s own tests**
(`header.rs:3187-3197`, `tests.rs:10766-10782`). The offset is a published
promise with no current consumer. Treat it as part of the wire format anyway —
it is exported, so a fork may be using it — but do not go looking for the
monitor code that toggles it, because there is none here.

## The checklist

1. **Decide the bump** against the table above. Write the reasoning into the
   `TOPIC_VERSION` doc comment in the same shape as the v4 entry
   (`header.rs:27-65`): what changed, why it is not detectable in-band, and what
   a mixed pair of processes does. That comment is the only record of v2 and v3,
   and it is already missing the `messages_total` move — do not add a second
   omission to it.
2. **Move the constant, not a copy.** Every offset lives in `shm_layout`. If you
   are about to write a byte number anywhere else, add a constant instead.
3. **Add the `offset_of!` assertion** for any new or moved field to
   `shm_layout.rs:281-291`. A constant with no assertion is exactly the
   arrangement that failed before.
4. **Build.** `cargo build -p horus_core` is enough to evaluate the whole
   `static_asserts` block; you do not need to run tests to catch an offset
   error.
5. **Bump `TOPIC_VERSION`** if step 1 said so, keeping the one-line
   `... const TOPIC_VERSION: u32 = N;` shape, and bump
   `CLI_TOPIC_VERSION` in `horus_manager/src/version.rs:24` in the same commit.
   `cli_topic_version_matches_horus_core` fails otherwise, and a stale mirror
   reports "compatible" for exactly the break it exists to catch.
6. **Walk the two tables above.** For each file in "carries the layout or the
   number", ask whether it needs to consult the thing you added, not only
   whether it still compiles.
7. **Run the layout suites.**
   - `cargo test -p horus_core --lib -- communication::topic` — covers the
     `shm_layout` unit tests (`shm_layout.rs:329-401`) and the header tests,
     including the two `verbose` offset tests and
     `messages_total_is_off_every_per_recv_line` (`header.rs:3392-3425`).
   - `cargo test -p horus_core --test pod_ring_slot_geometry -- --test-threads=1`
     — the command the file itself names (`:38`). It reads the backing file byte
     for byte and checks both geometries.
     `a_large_pod_type_stays_on_the_split_layout` (`:178`) is the control:
     without it, a build that co-located everything would satisfy every other
     test in the file.
   - `cargo test -p horus_net` — the `ShmRingWriter` tests
     (`horus_net/src/shm_writer.rs:306-522`).
   In CI these are reached by `integration-tests.yml:127`
   (`cargo test --workspace --exclude horus_py --release --test '*'`, plus
   `--test-threads=1 --nocapture` and five unrelated `--skip`s) and by
   `ci.yml:203` (`cargo test --workspace --exclude horus_py --lib
   --no-fail-fast`, plus four unrelated `--skip`s). Both jobs run on every pull
   request to `main`/`dev` with no `if:` guard.
8. **Check the cross-language and cross-process paths** if you touched slot
   geometry: `cargo test -p horus_core --test cross_language_ipc -- --test-threads=1`.
   Before reading anything into a failure there, confirm which Python extension
   the child actually loaded — a stale in-tree `.so` built against a different
   `TOPIC_VERSION` silently wins and the symptom is a clean zero
   (`horus_core/tests/cross_language_ipc.rs:54-58`; print `ext.__file__` from
   the child, `:60-67`).
9. **If you added a field a writer must consult**, say so in the `shm_layout`
   module doc, next to the readiness-lives-in-two-places note
   (`shm_layout.rs:38-50`). That paragraph exists because the two-places rule was
   the part everyone got wrong.

## What this procedure does not protect you from

Written down rather than implied, because a checklist that reads as complete is
worse than one that names its holes.

**`ShmRingWriter` does not read `layout_kind`.** Grepping `horus_net` for
`layout_kind`, `LAYOUT_COLO` and `colo` returns nothing. Reading the code:
`open_path` computes `stride = if is_pod { type_size } else { slot_size }` and
validates against `required_region_len_checked` (`shm_writer.rs:113-117`), and
`write_pod` addresses the slot with `data_slot_offset` and the ready word with
`seq_slot_offset` (`shm_writer.rs:201`, `:214`) — the split geometry,
unconditionally. A colo region has no sequence array, strides at
`colo_slot_size` (`shm_layout.rs:224`), and stores that stride as its
`slot_size` (`mod.rs:1009-1019`).

The length check does not stop it, and the arithmetic is checkable on paper.
`colo_eligible` caps `type_size` at 56 (`shm_layout.rs:206`, `:213-215`), so
`COLO_PAYLOAD_OFF + type_size <= 64 = colo_slot_size(type_size)`. The split
requirement `640 + capacity*8 + capacity*type_size` is therefore never larger
than the colo region's real size `640 + capacity*64`. Every colo region passes
`open_path`, and `write_pod`'s own `slot_end > mmap.len()` guard (`:209`) passes
for the same reason. `horus_core`'s own equivalent writer does read the byte and
branch (`horus_core/src/communication/mod.rs:252-253`), as do `read_slot_inner`
and `slot_stamp` (`header.rs:1517-1518`, `:1716-1722`); the `horus_net` writer
is the one that does not. The condition to reach it is a POD topic of 1 to 56
bytes that `horus_net` imports. Every `ShmRingWriter` test builds its fixture
with `make_pod_region` (`shm_writer.rs:320-336`), which never touches
`OFF_LAYOUT_KIND`, so it is always the split layout and the suite cannot see
this. **The reasoning above is read off the source; it has not been confirmed by
running anything.** Confirming it needs a test that creates a real `Topic<T>` of
a small POD type and points `ShmRingWriter::open_path` at its backing file. It
is the same shape as the original drift and should be settled before the next
release.

**`fuzz_shm_header.rs` is a second stale copy, and it runs nowhere.** It claims
to replicate "the exact offset arithmetic and bounds checking from header.rs"
(`horus_core/fuzz/fuzz_targets/fuzz_shm_header.rs:4-6`) and then computes the
POD slot as `HEADER_SIZE + last_written * type_size` (`:69`), omitting the
sequence array — the original v3 bug, preserved. It knows nothing of
`layout_kind`. Its sibling `fuzz_topic_header.rs` shows the shape it should have
had: that one writes the fuzzer's bytes to a file and calls the real
`read_latest_slot_bytes` (`:19-35`), copying nothing. No workflow invokes
either: the only fuzz job in CI is `cpp-fuzz`
(`.github/workflows/cpp-bindings.yml:595-646`), which runs with
`working-directory: horus_cpp` against four `horus_cpp/fuzz` targets, and
`horus_core/fuzz` is a separate workspace (`horus_core/fuzz/Cargo.toml:21-23`)
that nothing under `.github/workflows/` or `scripts/` names. Either wire it up
and make it use `shm_layout`, or delete it; leaving it is a copy of the layout
with no link to the layout, which is precisely what `shm_layout` was written to
abolish.

**Two sites recompute the geometry by hand instead of calling the module.**
`read_slot_inner`'s split branch derives `data_region_start` inline
(`header.rs:1602-1603`, `:1608`) and its serde branch uses literal `+ 8` and
`+ 16` for the length and data offsets (`header.rs:1624-1625`), where
`data_slot_offset`, `SERDE_SLOT_LEN_OFF` and `SERDE_SLOT_DATA_OFF` exist. The
numbers agree today. They are not bound by the `offset_of!` assertions, so they
would not agree automatically after a change to the serde slot shape. Both
branches also size the region with unchecked multiplication (`header.rs:1604`,
`:1618`) where the module supplies `required_region_len_checked`
(`shm_layout.rs:168`) for exactly the untrusted-header case this is — the colo
branch two lines above already uses the module's helpers (`:1592-1597`), so the
split and serde branches are the outliers, not the pattern.

**Nothing checks that this document is still true.** The parity-coverage
argument applies to it as much as to anything: a suite that is green tells you
what it asserts, not what it does not. The check worth adding is the one the
first table makes possible — assert that the `TOPIC_VERSION` literal quoted here
equals the constant, and that every file outside `horus_core` mentioning
`TOPIC_VERSION` or an `OFF_*` constant appears in that table — so that adding a
new out-of-crate reader without documenting it fails rather than passes. The
`horus-docs` copies at `installation.mdx:891` and `troubleshooting.mdx:426` are
in reach of the same check and are bound by nothing today.
