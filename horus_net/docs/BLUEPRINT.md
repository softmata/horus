# horus_net protocol blueprint

Thirteen files in `horus_net` cite a blueprint for their normative behaviour —
`//! See blueprint section 9.` and twelve like it. The document they cite is not
in this repository. A citation that resolves to nothing is worse than no
citation: it tells the next contributor that the rule they are about to change
was written down and agreed somewhere, so they go looking instead of reading the
code, and when they cannot find it they assume they are the ones who are lost.

## What was searched for, and what turned up

This matters because the next person will search too, and should not repeat it.

- No `horus_net/docs/` directory exists, and none ever has: enumerating every
  path ever added on any branch (`git log --all --diff-filter=A --name-only`)
  returns exactly `horus_net/Cargo.toml` and `.rs` files under `horus_net/`.
- No Markdown file in the history has ever contained the phrases a blueprint
  with these sections would have to contain. `git log --all -S'blueprint section
  21' -- '*.md'`, the same for `SC15`, and the same for
  `MAX_FRAGMENTS_PER_MESSAGE`, all return nothing.
- Four files matching `*BLUEPRINT*.md` do appear in the history, and one of them
  is a near-miss worth naming so nobody chases it twice:
  `horus_core/src/communication/network/NETWORK_V2_BLUEPRINT.md`, deleted in
  commit `8c923f2d` ("refactor network in horus_core"). It is 431 lines about a
  *different* thing — `sendmmsg`/`recvmmsg` batching, io_uring, QUIC, a
  transport selector — organised as Phases 1 to 5. It has no sections 6 to 21
  and no success criteria. It is not the document these thirteen files cite.

So: this file is that document, and it is derived **from the code that cites
it**, not from the design it was originally meant to record. That direction
matters and it changes what you may do with this file. Where the code and this
file disagree, the code is right and this file is a bug. This is a description
of a protocol, promoted to a specification because the protocol is already
deployed and its wire format is already load-bearing between machines — not a
design handed down to an implementation.

Section numbers here match the citations exactly, so that no citation dangles.
The numbering has gaps, and the gaps are real: see "Sections that nothing cites"
below.

## How to read the status marks

Every subsection carries one of two marks, because a large part of what a
contributor needs from this file is knowing which sentences describe running
code and which describe an intention that was typed into a doc comment and never
finished.

| Mark | Meaning |
|---|---|
| **Normative** | The shipped code does this. Changing it changes behaviour on the wire or on a robot. |
| **Aspirational** | Declared in the source — a type, a flag, a config field, a trait — and not delivered by any shipped path. Named here so nobody plans around it. |

There is a lot of the second kind. That is the honest state of the crate, and
the point of writing it down is that a reader who needs, say, packet batching or
readable metrics finds out here rather than after a day of instrumenting.

## Section index

| § | Subject | Module | Citation |
|---|---|---|---|
| 6 | Wire layout | `wire.rs` | `wire.rs:4` |
| 7 | Encoding and byte order | `encoding.rs` | `encoding.rs:3` |
| 8 | Reliability tiers | `reliability.rs` | `reliability.rs:7` |
| 9 | Import/export guard | `guard.rs` | `guard.rs:8` |
| 10 | Safety heartbeat | `heartbeat.rs` | `heartbeat.rs:6` |
| 11 | Fragmentation and reassembly | `fragment.rs` | `fragment.rs:7` |
| 12 | Optimizers | `optimize/mod.rs` | `optimize/mod.rs:4` |
| 13 | Priority | `priority.rs` | `priority.rs:3` |
| 14 | Flow control | `flow_control.rs` | `flow_control.rs:6` |
| 15 | Metrics | `metrics.rs` | `metrics.rs:6` |
| 17 | Discovery | `discovery.rs` | `discovery.rs:5` |
| 18 | Replication loop | `replicator.rs` | `replicator.rs:9` |
| 21 | Phase 1 success criteria | `tests/phase1_validation.rs` | `tests/phase1_validation.rs:2` |

---

## Section 6 — Wire layout

**Normative.** All framing is hand-written byte layout with no serde and no
external crate (`wire.rs:3`). Every multi-byte field is little-endian on the
wire regardless of host order; cross-endian handling is section 7's job and
applies to the *payload* only, never to the headers.

Constants: `MAGIC = 0x484F5253` ("HORS", `wire.rs:20`), `VERSION = 1`
(`wire.rs:23`), `MAX_UDP_PAYLOAD = 65507` (`wire.rs:26`).

### PacketHeader — 12 bytes

`PacketHeader::SIZE` is 12 (`wire.rs:46`). Offsets are fixed by
`encode`/`decode` at `wire.rs:59-92`.

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 4 | `magic` | `0x484F5253`; `decode` returns `None` on mismatch (`wire.rs:73-76`) |
| 4 | 1 | `version` | 1; `decode` returns `None` on mismatch (`wire.rs:77-80`) |
| 5 | 1 | `flags` | `PacketFlags` bitfield, below |
| 6 | 2 | `sender_id_hash` | truncated peer id (`wire.rs:40`) |
| 8 | 4 | `packet_sequence` | per-sender monotonic, for loss estimation (`wire.rs:41-42`) |

`PacketFlags` bits (`wire.rs:102-111`):

| Bit | Mask | Name | Status |
|---|---|---|---|
| 0 | `0x01` | `BATCH` | **Aspirational.** Set only in tests; see below. |
| 1 | `0x02` | `DELTA` | **Aspirational.** Declared at `wire.rs:103`; never set on any packet and never branched on. |
| 2 | `0x04` | `COMPRESSED` | **Aspirational.** Declared at `wire.rs:104`; same. |
| 3 | `0x08` | `FRAGMENT` | Normative — set by the export path at `replicator.rs:833-838`. |
| 4 | `0x10` | `HEARTBEAT` | Normative — `wire.rs:356`. |
| 5 | `0x20` | `ACK` | Normative — `wire.rs:383`. |
| 6 | `0x40` | — | Reserved, unused (`wire.rs:108`). |
| 7 | `0x80` | — | **Reserved as the announcement discriminator.** Packet encoders MUST NOT set it (`wire.rs:108-111`). See section 17. |

On `DELTA` and `COMPRESSED`: grepping both names across `horus_net/src` returns
their declarations (`wire.rs:103-104`), their accessors (`wire.rs:129-135`), and
one unit test that asserts both are clear (`wire.rs:708-709`). Nothing sets
them and no branch reads them.

Bit 7 is the one rule in this section you must not break. `AnnouncementHeader`
and `PacketHeader` share `MAGIC` and `VERSION` and both carry their flags byte
at offset 5, so without a discriminator every data, heartbeat, ack, fragment and
e-stop packet of 30 bytes or more false-matched `decode_announcement` and was
swallowed on the receive path (`discovery.rs:129-139`).

### MessageHeader — 24 bytes

`MessageHeader::SIZE` is 24 (`wire.rs:181`). Offsets are fixed by
`encode`/`decode` at `wire.rs:184-213`.

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 4 | `topic_hash` | FNV-1a of the **topic name** (`wire.rs:222-237`); the field's own doc comment at `wire.rs:159` says "topic type name" and is wrong — see "Stale comments in the source" |
| 4 | 4 | `payload_len` | |
| 8 | 8 | `timestamp_ns` | |
| 16 | 4 | `sequence` | per-topic monotonic |
| 20 | 1 | `priority` | `Priority as u8`, section 13 |
| 21 | 1 | `reliability` | `Reliability as u8`, section 8 |
| 22 | 1 | `encoding` | `Encoding as u8`, section 7 |
| 23 | 1 | `source_host` | low byte of the sender's peer id hash (`wire.rs:173-177`) |

Two caveats on this header, both load-bearing:

- **There is no type-identity field and no room for one.** All 24 bytes are
  allocated. The receiver's type check therefore has to use the hash the *peer
  announced in discovery*, which is why `replicator.rs:655` reads
  `peers.announced_type_hash` rather than anything on the packet.
- **`source_host` is written as 0 on every data packet.** `encode_single`
  (`wire.rs:465`), `encode_fragment` (`wire.rs:515`) and `encode_batch`
  (`wire.rs:546`) all hard-code it, and `decode_packet` builds an `InMessage`
  that has no `source_host` field at all (`wire.rs:417-425`, `wire.rs:601-609`),
  so even a non-zero value would be discarded. The only writer of a real value
  is the system-topic framer at `replicator.rs:1165`. The field's documented
  purpose — letting a receiver tell which host published a message — is
  **aspirational**.

### Trailing headers and special payloads

| Struct | Size | Present when | Source |
|---|---|---|---|
| `FragmentHeader` | 12 | `flags.FRAGMENT` | `wire.rs:255`; layout `wire.rs:257-263` |
| `HeartbeatPayload` | 20 | `flags.HEARTBEAT` | `wire.rs:291`; `peer_id[16]` + `heartbeat_sequence:u32` |
| `AckPayload` | 8 | `flags.ACK` | `wire.rs:325`; `acked_topic_hash:u32` + `acked_sequence:u32` |

Resulting packet shapes:

- single message: `[PacketHeader 12][MessageHeader 24][payload]`, 36 bytes of
  overhead (`wire.rs:15`, asserted at `wire.rs:659-662`)
- fragment: `[12][24][FragmentHeader 12][chunk]`, 48 bytes of overhead
  (`wire.rs:493`)
- heartbeat: 32 bytes total (`wire.rs:281`, `wire.rs:353`)
- ack: 20 bytes total (`wire.rs:315`, `wire.rs:380`)

### Encoder contract

`encode_single` and `encode_fragment` return **0** when the buffer is too small
and print a line naming the dropped message; callers must treat 0 as "not
encoded" and skip the send (`wire.rs:429-453`, `wire.rs:476-503`; honoured at
`replicator.rs:872-875` and `replicator.rs:978-980`). They used to `assert!`.
The workspace release profile sets only `lto = "thin"` (workspace root
`Cargo.toml:119-120`) — `grep -rn 'panic *= *"abort"' --include=Cargo.toml`
across the tree matches nothing — and `Replicator::run` is
spawned with no `catch_unwind` (`lib.rs:156-161`), so the assert unwound the
single `horus-net` thread and left the process alive with replication —
including e-stop — permanently dead, while `ReplicatorHandle::is_running()` kept
returning true (`wire.rs:434-441`). If you add an encoder, give it the same
contract.

`encode_batch` (`wire.rs:531-556`) has **no** such check: it `debug_assert!`s and
writes. Every call site is a test — `wire.rs:829`, `wire.rs:921`,
`replicator.rs:1546` (all `#[cfg(test)]`), `tests/wire_integration.rs:85`,
`tests/e2e_replication.rs:207` — and the same is true of every use of the
`BATCH` flag. Batching is therefore **aspirational**: the shipped export path
emits one message per datagram (`replicator.rs:861-871`). Before wiring
`encode_batch` into production, give it the return-0 contract.

`decode_packet` (`wire.rs:560-613`) returns early with no messages for heartbeat
and ack packets (`wire.rs:564-566`), then walks `[MessageHeader][payload]` pairs
until the buffer runs out. It does not require the `BATCH` flag to do so, which
is why that flag is decorative today. The length arithmetic uses `checked_add`
and `slice::get` rather than `+` and indexing (`wire.rs:590-596`): `usize` is 32
bits on the armv7 controllers this middleware targets — a target CI
cross-compiles the whole workspace for, `horus_net` included
(`.github/workflows/multi-platform.yml:275-329`) — and the release profile has
no `overflow-checks`, so a declared `payload_len` near `u32::MAX` wrapped the sum
down, passed the bounds guard, and panicked the replicator thread on one datagram
from anyone the source filter admits.

---

## Section 7 — Encoding and byte order

**Normative.** The `encoding` byte in each `MessageHeader` declares how the
payload should be read. Three values (`priority.rs:135-142`):

| Value | Name | Meaning |
|---|---|---|
| 0 | `PodLe` | raw POD bytes, little-endian |
| 1 | `PodBe` | raw POD bytes, big-endian |
| 2 | `Bincode` | serde bincode; handles its own byte order |

Every one of the three wire enums maps an unrecognised byte to a fixed value,
and they do not agree on which. `Encoding::from_u8` falls back to `Bincode`
(`priority.rs:145-152`), `Priority::from_u8` to `Normal` (`priority.rs:21-29`)
and `Reliability::from_u8` to `None` (`priority.rs:101-108`). Each is the
conservative answer for its own enum — the one that neither claims an
interpretation of the bytes nor promises delivery — but do not assume a common
rule.

`check_encoding` (`encoding.rs:23-30`) yields `SameEndian` (memcpy, zero
overhead), `ByteSwapped` (swapped in place) or `Bincode` (left alone).
`process_incoming_payload` (`encoding.rs:69-80`) is the single entry point and
is called once per imported message at `replicator.rs:673`.

Two limits you should know before relying on the cross-endian path:

- The swap is a uniform-word-size reversal chosen by a divisibility heuristic:
  8 if the size divides by 8, else 4, else 2, else 1 (`encoding.rs:54-64`), and
  the source calls it best-effort for mixed-width structs (`encoding.rs:34-39`).
  A struct of mixed field widths is swapped wrongly and nothing detects it.
- The size handed to that heuristic is the **payload length**, not the
  registered type size: `replicator.rs:671-673` passes `payload.len()`. For a
  POD topic those are equal, so this is currently harmless — but it means the
  heuristic is driven by a wire-supplied length rather than by local type
  knowledge.

---

## Section 8 — Reliability tiers

**Normative.** Three tiers (`priority.rs:91-98`), each a `u8` on the wire:

| Value | Tier | Copies sent | Behaviour |
|---|---|---|---|
| 0 | `None` | 1 | fire and forget |
| 1 | `Redundant` | 2 | staggered copies, no ack |
| 2 | `Latched` | 3 | copies **and** resend until acked |

`ReliabilityLayer::copies_for` is the authority on the copy counts
(`reliability.rs:54-60`). The default tier for a priority is
`Reliability::default_for` (`priority.rs:122-129`), re-exported as
`ReliabilityLayer::default_reliability` (`reliability.rs:63-65`) and locked by
`default_reliability_matches_blueprint` (`reliability.rs:188-206`):

| Priority | Default reliability |
|---|---|
| `Immediate` | `Latched` |
| `RealTime` | `Redundant` |
| `Normal` | `None` |
| `Bulk` | `None` |

Latching: a latched message is tracked by `(topic_hash, sequence)`
(`reliability.rs:34`) and resent every `LATCH_RESEND_INTERVAL` = 10 ms
(`reliability.rs:19`) until an ack arrives, up to `MAX_LATCH_RESENDS` = 100
(`reliability.rs:16`), after which the entry is dropped with a console line
(`reliability.rs:98-103`). Resends are driven from the timer path and go to
**every** alive peer, not only the original destination
(`replicator.rs:959-987`).

**Latching applies only to the data path.** `start_latch` has exactly one
non-test caller, inside `handle_export` (`replicator.rs:891-894`).
`Reliability::for_system_topic` marks `_horus.estop` as `Latched`
(`priority.rs:111-119`), and the framer stamps that byte onto the wire
(`replicator.rs:1142`), but no latch is ever opened for it and the receiver
returns from the system-topic dispatch (`replicator.rs:618`) long before the ack
block (`replicator.rs:720`). E-stop redundancy comes from `EstopBroadcaster`,
not from this tier — see section 18. The `Latched` byte on a system topic is
inert.

Acking, on the receive side, is conditional on two things beyond the reliability
byte (`replicator.rs:720`): the message must have been **accepted** into local
SHM, and the source must be a peer already in the peer table. Acking on the byte
alone acked messages this node had discarded — cancelling the sender's latch on
the one tier that exists to make silent loss impossible — and turned the node
into a packet-count amplifier, because a 65507-byte datagram of zero-length
messages draws ~2700 separate 20-byte acks at a spoofable source address
(`replicator.rs:693-719`).

Deduplication: `is_new_message` keys on `(sender_id_hash, topic_hash)` and drops
anything at or below the high-water sequence, with a wrapping tolerance of
`u32::MAX / 2` (`reliability.rs:120-132`). The map is capped at
`netfilter::MAX_TRACKED_KEYS` (`reliability.rs:37-39`, `netfilter.rs:300`).
**System topics are never deduplicated** (`reliability.rs:135-161`): the key's
sender half is a 16-bit cleartext value nothing authenticates, and dedup runs
upstream of the e-stop MAC check, so one forged packet carrying a victim's
sender hash, the `_horus.estop` hash and a high sequence permanently silenced
genuine remote e-stop from that peer. If you touch this filter, that exemption
stays.

---

## Section 9 — Import/export guard

**Normative, with one documented contradiction.**

Import modes (`guard.rs:16-24`) and export modes (`guard.rs:27-35`):

| Import mode | Effect |
|---|---|
| `Deny` | no remote data enters local SHM |
| `Auto` | import a topic this process subscribes to and does **not** publish |
| `AllowList(patterns)` | glob allowlist |

| Export mode | Effect |
|---|---|
| `All` | export everything with a remote subscriber |
| `AllowList(patterns)` | glob allowlist |
| `DenyList(patterns)` | everything except the patterns |

**The shipped import default is `Auto`, not deny.** `guard.rs:3-4` states
"Import default: deny all", and that is wrong: `ImportExportGuard::new_default`
uses `ImportMode::Auto` (`guard.rs:47-53`), `ImportConfig`'s `#[default]` is
`Auto` (`config.rs:95-96`), and `ImportConfig::from_env` returns `Auto` when
`HORUS_NET_IMPORT` is unset (`config.rs:111-116`). `config.rs:82-85` says so
plainly. Treat the module header as stale prose; treat this paragraph and
`config.rs` as the specification. `HORUS_NET_IMPORT=deny` is the setting that
keeps a remote host out of local SHM. All three import modes are reachable from
configuration (`replicator.rs:146-150`).

`Auto` resolves against the registry role (`guard.rs:92-103`): `Subscriber`
imports, `Publisher` and `Both` do not, and an unknown topic does not. The role
distinction is only real because `horus_core` reports direction on a handle's
first send or first recv (`TopicLifecycleEvent::RoleObserved`, `lib.rs:119-137`);
the lifecycle hook alone only learns that a topic was *created*, and registering
everything as `Both` broke this rule in both directions — first denying 100% of
imports silently, then, once patched by treating `Both` as importable, letting a
remote peer overwrite commands this robot produces itself (`guard.rs:80-91`).

The residual risk is documented rather than eliminated: a topic is registered as
`Subscriber` at construction and only upgraded to `Both` on its first local send
(`lib.rs:101-118`), so a topic this node will publish is importable for one node
tick. The source argues that window is inside the peer's one-second announce
cycle.

System topics: `allow_import` returns true for them unconditionally
(`guard.rs:71-73`), which is why source admission control has to run before any
of this (see section 18). `allow_export` **does** consult the export mode for
them (`guard.rs:122-128`): they used to bypass it outright, so `_horus.logs` —
every Error and Warning this node emits — and `_horus.presence` were exported to
any host that announced an interest and an operator's `deny_export` entry was
silently ignored.

Globbing is `guard.rs:146-200`: `*` alone matches everything, one `*` is
prefix/suffix, several `*` must appear in order.

**Aspirational:** `ExportMode::AllowList` is unreachable from configuration.
`Replicator::new` builds either `All` or `DenyList` and nothing else
(`replicator.rs:151-155`); there is no allow-export config key.

---

## Section 10 — Safety heartbeat

**Normative.** Separate from discovery. Discovery is 1 Hz multicast; the safety
heartbeat is bidirectional direct UDP between matched peers at 20 Hz
(`heartbeat.rs:1-4`).

Defaults: 50 ms interval, 3 missed beats, `Warn` action (`heartbeat.rs:96-106`);
in production these come from `NetConfig` (`replicator.rs:159-172`). Link-lost
actions (`heartbeat.rs:18-25`), parsed by `LinkLostAction::from_str`
(`heartbeat.rs:29-35`):

| Action | Config string | Effect |
|---|---|---|
| `Warn` | anything unrecognised, including the default `warn` | log only |
| `SafeState` | `safe_state` / `safestate` | `trigger_external_safe_state` (`replicator.rs:939-949`) |
| `Stop` | `stop` | `trigger_external_emergency_stop` (`replicator.rs:950-955`) |

`SafeState` and `Stop` are genuinely different destinations. Routing both to the
emergency stop gave an operator who chose per-node safing the full halt and
preserved their choice only in the log line (`replicator.rs:939-944`).

Three rules in `tick` (`heartbeat.rs:223-300`) that exist because each was once
wrong:

1. **Divide in nanoseconds.** `interval.as_millis()` is 0 for a sub-millisecond
   interval and integer division by zero panics the replicator thread, killing
   networked e-stop for a configuration that is aggressive rather than invalid
   (`heartbeat.rs:248-259`).
2. **Declare the link dead on the Nth missed beat, not the N+1th.** The
   threshold field says "number of missed heartbeats before declaring link
   dead", so the comparison is `>=` (`heartbeat.rs:261-264`, `heartbeat.rs:282`).
3. **Do not judge a peer that has never spoken.** `add_peer` starts the clock
   the moment discovery sees an announcement, but the far side only learns we
   exist on its own 1 s announce cycle. At the default 50 ms × 3 that declared a
   perfectly healthy peer lost about 150 ms after meeting it — measured at 196 ms
   on loopback with two real replicators — and with `on_link_lost = "stop"` it
   halted a working robot during fleet assembly. A peer is judged only once it
   has answered, or once `DISCOVERY_GRACE` = 2 s has passed
   (`heartbeat.rs:88-93`, `heartbeat.rs:265-282`).

Table bounds: `add_peer` is capped at `netfilter::MAX_PEERS` = 256
(`heartbeat.rs:136-148`, `netfilter.rs:307`), and its caller registers a peer
only when `PeerTable::update_peer` returned true (`replicator.rs:423-425`,
`peer.rs:94-148`). Both are required. Registering unconditionally into an
uncapped table meant one forged announcement bought a heartbeat transmitted every
50 ms, forever, at an address the sender chose. `retain_peers`
(`heartbeat.rs:168-177`) re-syncs the table against `PeerTable` each discovery
interval (`replicator.rs:1016-1022`), because `remove_peer` only fires for a
peer reported newly dead and a peer evicted to make room is never reported that
way.

RTT is an EMA with alpha 0.2 over the time since our last send
(`heartbeat.rs:190-199`), readable through `rtt_us` (`heartbeat.rs:216-218`).
Nothing forwards it into `metrics` — see section 15.

---

## Section 11 — Fragmentation and reassembly

**Normative.** Bounds (`fragment.rs:15-37`):

| Constant | Value | Purpose |
|---|---|---|
| `MAX_FRAGMENT_PAYLOAD` | 1400 | split threshold and chunk size |
| `MAX_REASSEMBLY_SIZE` | 1 048 576 | largest message the crate carries |
| `MAX_FRAGMENTS_PER_MESSAGE` | `MAX_REASSEMBLY_SIZE / MAX_FRAGMENT_PAYLOAD + 1` = 749 | bounds a `vec![None; count]` sized from the wire |
| `MAX_PENDING_MESSAGES` | 256 | concurrent partial messages |
| `FRAGMENT_TIMEOUT` | 100 ms | stale partial discard |

A message at or under 1400 bytes is returned as one `Fragment` with
`fragment_count = 1` and is sent by `encode_single` with no `FragmentHeader`
(`fragment.rs:78-94`, `replicator.rs:861-871`). Over `MAX_REASSEMBLY_SIZE` the
message is dropped with a console line pointing at horus-zenoh
(`fragment.rs:96-105`).

**A fragmented message MUST carry a `FragmentHeader`.** The send path once called
`encode_single` for every fragment, which emits none, while the receive path
reads one at `PacketHeader::SIZE + MessageHeader::SIZE` — so every fragmented
message was parsed with the first 12 bytes of its chunk read as fragment
metadata, and nothing over 1400 bytes ever reassembled (`wire.rs:481-486`,
`replicator.rs:855-860`).

**The reassembly key includes the sender.** It is
`(sender_id_hash, topic_hash, fragment_id)` (`fragment.rs:173-181`,
`fragment.rs:222`, `replicator.rs:470-476`). `fragment_id` is a per-process
counter every `Fragmenter` starts at 0 (`fragment.rs:69-72`), so keyed on
`(topic_hash, fragment_id)` alone two robots publishing the same topic
reassembled into one buffer and a message was spliced together from both. Slot
filling is first-write-wins, so the genuine fragment was discarded as a
duplicate. This needed no attacker.

`Reassembler::feed` (`fragment.rs:197-298`) applies four bounds in order and each
fails closed by returning `None`: declared total against the cap; fragment count
against `MAX_FRAGMENTS_PER_MESSAGE`, checked **before** the slot vector is
allocated from it; concurrent partial count against `MAX_PENDING_MESSAGES`, which
refuses new keys but still feeds existing ones; and a running byte sum against
the declared total, which drops the whole partial message on overflow rather than
retaining attacker state. `gc_stale` (`fragment.rs:302-307`) is driven from the
timer at `replicator.rs:990`.

---

## Section 12 — Optimizers

**Normative in structure; largely aspirational in effect.** The rule that
matters: all optimizers are off by default and `Immediate` priority always
bypasses the whole chain (`optimize/mod.rs:3`). The bypass is not a per-optimizer
courtesy — `process_outgoing` extracts `Immediate` messages before the chain and
re-appends them first (`optimize/mod.rs:89-113`). Incoming runs the chain in
reverse (`optimize/mod.rs:116-125`). An empty chain returns immediately
(`optimize/mod.rs:90-92`, `optimize/mod.rs:117-119`).

### The chain is a closed match, not an open registry

`OptimizerChain::from_config` (`optimize/mod.rs:52-78`) is a four-arm match on
literal names. `Optimizer` is a public trait (`optimize/mod.rs:19-36`) and `add`
takes any `Box<dyn Optimizer>` (`optimize/mod.rs:81-83`), so an optimizer can be
installed in-process — but every non-test call to `add` is inside that match, and
there is no registration hook. **Adding an optimizer means editing
`optimize/mod.rs:55-75` by hand.** Say so when you plan work; "pluggable" in the
trait's own doc comment is about the trait, not about the deployment.

| Name | Installed? | Source |
|---|---|---|
| `fusion` | yes | `optimize/mod.rs:56` |
| `delta` | **refused**, with a console line | `optimize/mod.rs:66-69` |
| `spatial` | yes | `optimize/mod.rs:70` |
| `predict` | yes | `optimize/mod.rs:71` |
| anything else | warns, not installed | `optimize/mod.rs:72-74` |

`delta` is refused on purpose. Only the encode half exists: `on_outgoing`
replaces the payload with a region stream, `on_incoming` is a no-op, and no wire
flag marks a packet as delta-encoded, so the receiver hands the region stream to
`ShmRingWriter::write` as the message body — a POD topic then fails the size
check and the remote subscriber silently freezes between keyframes, and a non-POD
topic deserializes garbage (`optimize/delta.rs:19-28`,
`optimize/mod.rs:57-65`). Finishing it needs a per-message delta bit on the wire,
and `MessageHeader` has no spare room (section 6). The unit test asserting delta
is installed was deliberately inverted (`optimize/mod.rs:236-243`). The
`horus.toml` schema carries the same warning at
`horus_manager/src/manifest.rs:296-297`.

### What the three installable optimizers actually do

- **`fusion`** buffers outgoing messages for a 500 µs window
  (`optimize/fusion.rs:16`, `optimize/fusion.rs:64-94`) and flushes them back
  into the vector when the window expires. It reduces no packets today, because
  the export path sends each message in its own datagram via `encode_single` /
  `encode_fragment` (`replicator.rs:861-871`) and never calls `encode_batch`.
  Its own doc comment claims otherwise (`optimize/fusion.rs:6-8`). **Aspirational**
  as a bandwidth measure; it currently only delays.
- **`spatial`** filters nothing on the message path: `on_outgoing` increments
  two counters and returns (`optimize/spatial.rs:97-104`). The comment at
  `optimize/spatial.rs:100` says the replicator calls `is_within_radius()` per
  peer; the replicator does not — every call to that method is in a test
  (`optimize/spatial.rs:123-182`, `tests/phase3_validation.rs:280`). Default
  radius 15.0 (`optimize/spatial.rs:41`). **Aspirational.**
- **`predict`** does work: it suppresses messages whose linear extrapolation is
  within `DEFAULT_THRESHOLD` = 0.01, after a `COLD_START_COUNT` = 5 warm-up and
  with an unconditional send every `MAX_SUPPRESS_COUNT` = 100 messages
  (`optimize/predict.rs:12-18`, `optimize/predict.rs:75-119`). **Normative.**

`Optimizer::max_priority` (`optimize/mod.rs:31-35`) has a default body and **no
caller anywhere in the crate** — grepping the name across `horus_net/src` and
`horus_net/tests` returns only that definition. A per-optimizer priority ceiling
is aspirational; the only priority gate that runs is the `Immediate` extraction
above.

---

## Section 13 — Priority

**Normative.** Four levels, `repr(u8)`, ordered so that lower is more urgent
(`priority.rs:6-17`):

| Value | Level | Intent |
|---|---|---|
| 0 | `Immediate` | e-stop; bypasses all optimizers; latched |
| 1 | `RealTime` | motor commands; redundant |
| 2 | `Normal` | sensor data; full pipeline; the default |
| 3 | `Bulk` | camera frames, logs |

`Priority::from_u8` maps unknown bytes to `Normal` (`priority.rs:21-29`).

Inference. `infer_from_topic` matches the lowercased name against `estop`,
`emergency` or `safety` and returns `Immediate`, else `Normal`
(`priority.rs:32-39`). `auto_infer` adds two rules in strict precedence — name
first, then a node RT budget, then size over 64 KiB, then `Normal`
(`priority.rs:44-59`):

| Condition | Result |
|---|---|
| topic name contains `estop`/`emergency`/`safety` | `Immediate` |
| node has an RT budget or deadline | `RealTime` |
| message over 64 × 1024 bytes | `Bulk` |
| otherwise | `Normal` |

**`RealTime` is unreachable on the export path.** `handle_export` calls
`auto_infer(&topic_name, false, raw.data.len())` — the budget argument is a
literal `false` (`replicator.rs:784`), and nothing plumbs a node's RT budget into
the replicator. In shipped code an exported message is `Immediate`, `Bulk` or
`Normal`. That in turn means `Reliability::Redundant` is never chosen
automatically either (section 8).

System topics take a fixed priority (`priority.rs:62-73`): `_horus.estop` is
`Immediate`, `_horus.logs` is `Bulk`, `_horus.presence` and everything else are
`Normal`. `Priority::from_str` accepts `immediate`, `realtime`/`real_time`/`rt`,
`normal`, `bulk`, and maps anything else to `Normal` (`priority.rs:77-85`).

**Aspirational:** per-topic priority and reliability overrides. `TopicNetConfig`
declares eight fields — `priority`, `reliability`, `redundant_copies`,
`on_link_lost`, `export_sampling`, `optimizers`, `spatial_radius` and
`predict_threshold` (`config.rs:204-224`). Seven of the eight have no reader
anywhere in the crate. The eighth, `export_sampling`, is read by
`NetConfig::export_sampling` (`config.rs:337-354`), which is the only non-test
reader of `topic_config` (`config.rs:314-326`) — but the map it reads,
`topic_overrides`, is initialised empty (`config.rs:261`, `config.rs:309`) and is
populated only by tests (`config.rs:549`, `config.rs:567`,
`tests/export_sampling.rs:312`, `tests/export_sampling.rs:327`). No `horus.toml`
key reaches it: the `[network]` table has exactly six keys and none is per-topic
(`horus_manager/src/manifest.rs:276-303`), and `apply_network_config` sets no
per-topic variable (`horus_manager/src/commands/run/mod.rs:79-142`). Per-topic
overrides are reachable only from Rust, by mutating `NetConfig` before
`Replicator::new`.

---

## Section 14 — Flow control

**Normative.** The module header still says "Phase 1: should_send() always
returns true" (`flow_control.rs:4`); that is stale — `should_send` does throttle
(`flow_control.rs:186-201`). The rest of this section is the current behaviour.

`on_received` records `(sender_id_hash, topic_hash)` → sequence and accumulates
gaps over a 1 s window (`flow_control.rs:12`, `flow_control.rs:107-136`). The map
is bounded by `netfilter::MAX_TRACKED_KEYS` = 4096, because both halves of the
key come off the wire before any guard runs (`flow_control.rs:88-93`).

Rate ladder (`flow_control.rs:33-55`; the comparisons are strict `<`, so exactly
1% loss is `Half` and exactly 20% is `KeyframeOnly`):

| Loss rate | `SendRate` | Sends |
|---|---|---|
| < 0.01 | `Full` | every message |
| < 0.05 | `Half` | every 2nd |
| < 0.20 | `Quarter` | every 4th |
| otherwise | `KeyframeOnly` | every 100th |

**This throttle fails open, deliberately, and there are three separate guards
making it do so.** Every input is unauthenticated and it decides whether a robot
publishes at all (`flow_control.rs:146-185`).

1. An absent estimate sends (`flow_control.rs:187-189`).
2. `estimate_is_usable` requires the window to be current **and** at least
   `MIN_SAMPLES_TO_DERATE` = 8 samples (`flow_control.rs:29`,
   `flow_control.rs:155-161`). Two datagrams — `seq = 1` then `seq = 1_000_000` —
   produced `gaps = 999_998, total = 2`, a loss rate of ~500, and
   `KeyframeOnly` for that pair; because the state only changes inside
   `on_received`, `total` then stayed at 2 forever and `2.is_multiple_of(100)` is
   false, so `should_send` returned false permanently and silently, from two
   forged packets. A closed window is likewise a measurement of the past.
3. The caller bypasses the throttle entirely for `Immediate` and `RealTime`
   (`replicator.rs:879-882`).

The lookup key is the **destination** peer's id hash, the same value that peer
stamps into its own packets (`flow_control.rs:172-177`, `peer.rs:42-50`,
`replicator.rs:825-830`). Passing our own hash silently disables the throttle.

And the record side is gated: `on_received` runs only for a datagram attributable
to a peer already known at that source address
(`replicator.rs:559-565`, via `peers.id_hash_announced_from`). Recording
unconditionally let any admitted host file a fabricated loss statistic under a
legitimate peer's 16-bit cleartext id and throttle this robot's sends on a topic
it cannot write — reaching the same subscriber with the same staleness the import
guard exists to prevent (`replicator.rs:542-558`).

---

## Section 15 — Metrics

**Normative in recording; aspirational in reporting.** Counters are plain
atomics with `Relaxed` ordering, internal to the `Replicator`
(`metrics.rs:1-6`).

Per peer (`metrics.rs:13-21`): `rtt_us`, `loss_rate_permille`, `bytes_sent`,
`bytes_received`, `packets_sent`, `packets_received`, `last_seen_ms`.
Per topic (`metrics.rs:25-37`): `messages_sent`, `messages_received`,
`messages_dropped`, `messages_skipped`, `bytes_sent`, `bytes_received`,
`last_sequence`. Plus a crate-level `type_mismatches` counter
(`metrics.rs:75`, `metrics.rs:88-95`).

`messages_dropped` and `messages_skipped` mean different things and the
distinction is the whole reason the second exists (`metrics.rs:29-33`):
`dropped` is an import-side refusal, `skipped` is local data that never left this
machine because latest-only export sampling decimated it or the ring lapped. A
topic replicating at 20 Hz instead of 500 Hz used to be indistinguishable from a
topic published at 20 Hz (`metrics.rs:131-136`). `record_topic_skipped` ignores a
zero delta so it cannot conjure a topic row (`metrics.rs:137-140`).

Two honest gaps:

- **Four fields are never written.** `rtt_us`, `loss_rate_permille`,
  `last_seen_ms` and `last_sequence` have no store anywhere in the crate;
  grepping each name across `horus_net/src` finds the declaration and, for the
  first two, one load in `snapshot`, so those two always report 0
  (`metrics.rs:152-153`). `last_seen_ms` and `TopicMetrics::last_sequence` are
  not even read — neither `PeerSnapshot` nor `TopicSnapshot` carries them
  (`metrics.rs:46-68`). `SafetyHeartbeat::rtt_us` (`heartbeat.rs:216-218`) and
  `FlowController::loss_rate` (`flow_control.rs:139-144`) do compute those
  numbers and nothing bridges them across.
- **Nothing outside the crate can read any of this.** `Replicator.metrics` is a
  private field (`replicator.rs:90`) with no accessor, `ReplicatorHandle` exposes
  only `stop` and `is_running` (`lib.rs:182-192`), and every caller of
  `NetMetrics::snapshot` is a unit test (`replicator.rs:1802`,
  `replicator.rs:1842` — both past the `#[cfg(test)]` at `replicator.rs:1297` —
  and `metrics.rs:196-252`, past the one at `metrics.rs:185`). There is no CLI
  command, no system topic and no API that surfaces them.

---

## Section 17 — Discovery

**Normative.** Every replicator joins multicast group `224.0.69.72:9100` and
announces its topics on a 1 s cycle (`discovery.rs:3-4`, `replicator.rs:41`).
Unicast to explicit peers is the alternative and skips multicast entirely
(`config.rs:280-293`).

### Announcement wire format

`AnnouncementHeader` — 30 bytes (`discovery.rs:157`), offsets at
`discovery.rs:159-168`:

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | `magic` (`0x484F5253`) |
| 4 | 1 | `version` (1) |
| 5 | 1 | `flags` — bit 7 `ANNOUNCEMENT_FLAG` required, bit 0 `ANNOUNCEMENT_HAS_SECRET` |
| 6 | 16 | `peer_id` |
| 22 | 2 | `data_port` |
| 24 | 4 | `secret_hash` |
| 28 | 2 | `topic_count` |

`ANNOUNCEMENT_FLAG` is `0x80` and `ANNOUNCEMENT_HAS_SECRET` is `0x01`
(`discovery.rs:140`, `discovery.rs:143`). `encode_announcement` always sets bit
7 (`discovery.rs:226`) and `AnnouncementHeader::decode` **requires** it
(`discovery.rs:187-189`). This is the same rule as section 6's bit 7, stated from
the other side.

`WireTopicEntry` — 76 bytes each (`discovery.rs:70`), offsets at
`discovery.rs:83-99`: `name_len:u8`, `name[63]` zero-padded, `type_hash:u32`,
`type_size:u32`, `role:u8` (1 = pub, 2 = sub, 3 = both), `is_pod:u8`,
`priority:u8`, one pad byte. At most
`MAX_TOPICS_PER_ANNOUNCEMENT` = 200 entries per announcement
(`discovery.rs:16`), so an announcement is at most 30 + 200 × 76 = 15 230 bytes.
The section comment above the struct says "72 bytes" (`discovery.rs:55`) and is
stale; `SIZE` and the offsets are the specification.

Three consequences worth knowing:

- **Topic names over 63 bytes do not replicate.** `MAX_TOPIC_NAME` is 63
  (`discovery.rs:13`); `encode` truncates (`discovery.rs:86-88`) and
  `find_matches` compares names for exact equality (`discovery.rs:322-324`), so
  the truncated name matches nothing. Truncation can also split a multi-byte
  character, in which case `decode` rejects that entry (`discovery.rs:109`) and
  `decode_announcement` skips it while still advancing the offset
  (`discovery.rs:273-281`).
- **The announced `priority` byte is written and never read.**
  `from_registry_entry` fills it from `infer_from_topic` — name-only, not
  `auto_infer` (`discovery.rs:79`) — `encode` puts it at byte 74
  (`discovery.rs:97`) and `decode` recovers it (`discovery.rs:114`,
  `discovery.rs:122`), and nothing anywhere consumes it. **Aspirational.**
- **`decode_announcement` sizes its allocation from the datagram, not from
  `topic_count`** (`discovery.rs:265-270`). Reserving from the wire-supplied u16
  let a 30-byte packet force a ~2.6 MB allocation, repeatable at line rate.

### Matching

`find_matches` (`discovery.rs:313-356`) pairs a local entry with a remote entry
of the same name. A type-hash mismatch produces a warning and no match. Then
`export = local_pub && remote_sub` and `import = local_sub && remote_pub`. This
is where replication is demand-driven: a topic nobody remote subscribes to is
never read out of SHM.

### Peer identity and liveness

`generate_peer_id` returns 16 bytes, and despite the comment at
`discovery.rs:23` it does **not** read `/dev/urandom`: it hashes
`SystemTime::now()`, the pid, the thread id and a stack address through
`DefaultHasher` (`discovery.rs:21-44`). It is a uniqueness device, not a
cryptographic one. `peer_id_hash` XOR-folds it to 16 bits for packet headers
(`discovery.rs:47-53`) — that is the `sender_id_hash` of section 6, and it is
attacker-choosable.

`secret_hash` is the FNV-1a u32 of `HORUS_NET_SECRET` in little-endian bytes
(`discovery.rs:359-361`, `config.rs:357-362`). The check fails **closed**: when
we hold a secret, an announcement must carry a matching one, and clearing the
has-secret bit no longer skips the check (`replicator.rs:400-414`). It is still
a cleartext, non-cryptographic filter against accidental cross-fleet mixing, not
authentication.

Discovery timers:

| Timer | Value | Source |
|---|---|---|
| Announcement interval | 1 s | `replicator.rs:41` |
| Event-loop timer tick | 50 ms | `replicator.rs:44` |
| No-peers diagnostic | 5 s, printed once per process | `replicator.rs:47`, `config.rs:391`, `config.rs:397-421` |
| Peer death | `interval × miss_threshold` = 3 s native, 6 s on WSL2 | `peer.rs:69-81`, `peer.rs:152-167` |
| Presence cleanup | 5 s | `replicator.rs:1120-1124` |
| Presence dead timeout | 30 s | `presence.rs:17` |

`PeerTable` is capped at `MAX_PEERS` = 256 and evicts the least recently seen
**dead** peer to make room; when every slot holds a live peer the announcement is
dropped and `update_peer` returns false (`peer.rs:104-133`). That return value is
load-bearing — see section 10.

**Known defect, not a rule:** `send_discovery_announcement` hard-codes port 9100
in the multicast address (`replicator.rs:1224`) while `multicast_addr` uses
`config.port` (`replicator.rs:1202-1209`). With `HORUS_NET_PORT` set to anything
else, announcements and system-topic multicast go to different ports.

---

## Section 18 — Replication loop

**Normative.** One `Replicator` per process, on one thread, started by the
scheduler (`replicator.rs:59`, `lib.rs:84`, `lib.rs:156-161`; the scheduler
`on_start` hooks are at `horus/src/lib.rs:217`, `:318` and `:340`).

### Event loop

Three sources, dispatched from a single `epoll`/`kqueue` wait
(`replicator.rs:277-301`, `event_loop/mod.rs:12-21`): the UDP socket (import),
an eventfd signalled by `TopicRegistry` (export), and a timerfd at
`TIMER_INTERVAL` = 50 ms. Unix only — there is no Windows backend and the
crate does not compile on one (`event_loop/mod.rs:1-2`,
`event_loop/mod.rs:29-41`). CI reflects that: the Windows job runs
`cargo check`/`cargo build --workspace --exclude horus_net`
(`.github/workflows/multi-platform.yml:140-150`).

Two lifetime rules in `run` that are easy to reintroduce:

- The export-notify callback owns a **dup** of the eventfd, not the raw number
  (`replicator.rs:239-275`). The callback is installed into a process-lifetime
  registry singleton and outlives the event loop, so every `Topic<T>` dropped
  during scheduler shutdown wrote 8 bytes to a descriptor number that may by then
  belong to an unrelated file.
- `clear_on_change()` runs before the loop's stack frame unwinds
  (`replicator.rs:303-307`).

### Import path

`handle_incoming` drains at most `MAX_DATAGRAMS_PER_WAKEUP` = 256 per wakeup
(`replicator.rs:330`, `replicator.rs:341`). Looping until the socket is empty
never terminates under a sustained inbound rate, and because the timer is a
sibling match arm reached only after this returns, a flood starved every periodic
safety action — outbound heartbeats, peer liveness, reassembly GC and the e-stop
drain (`replicator.rs:315-329`). The scratch buffers are owned fields, not stack
arrays: a zeroed 64 KiB stack array measured 1145 ns against 2.8 ns for a reused
buffer (`replicator.rs:104-119`).

`process_packet` dispatch order (`replicator.rs:354-524`) — the branches are
mutually exclusive only because of the bit-7 discriminator:

| Step | Test | Handler |
|---|---|---|
| 0 | `peer_filter.admits(from)` | drop, count, log once (`replicator.rs:381-392`) |
| 1 | announcement (flags bit 7) | peer table, heartbeat registration, rematch (`replicator.rs:396-435`) |
| 2 | `PacketFlags::HEARTBEAT` | `heartbeat.on_received` (`replicator.rs:444-449`) |
| 3 | `PacketFlags::ACK` | `reliability.on_ack` (`replicator.rs:452-457`) |
| 4 | `PacketFlags::FRAGMENT` | reassembly, then message dispatch (`replicator.rs:460-506`) |
| 5 | otherwise | `decode_packet`, dedup, incoming optimizers, message dispatch (`replicator.rs:509-523`) |

**Step 0 must stay first.** The socket binds `0.0.0.0` (`transport/udp.rs:3`,
`transport/udp.rs:24-25`), the system-topic dispatch returns before the import
guard, and `allow_import` returns true unconditionally for system topics — so
reordering the guard would not help; only source filtering here covers all five
packet kinds and all three system topics with one check
(`replicator.rs:367-380`). The default allow set is loopback, the three RFC 1918
blocks, RFC 3927 link-local and RFC 6598 CGNAT space (`netfilter.rs:90-102`).
`HORUS_NET_ALLOW_PEERS` widens or narrows it; an explicit list **replaces** the
defaults rather than adding to them (`netfilter.rs:157-178`, asserted at
`netfilter.rs:404-411`), the wide-open setting is reachable only by asking for it
by name — `any`, `*`, `all` or `0.0.0.0/0` — and warns loudly
(`netfilter.rs:145-155`), and a list whose entries are all unparseable falls back
to the default rather than to `Any`, because a typo must not quietly undo the
hardening (`netfilter.rs:171-177`).

`process_incoming_message` (`replicator.rs:536-734`) then runs, in order: flow
control recording (gated, section 14); system-topic dispatch; import guard; type
hash check; encoding; SHM write; conditional ack.

One resolution detail that matters when you read the guard: the topic name is
recovered by scanning the **writers** map for a matching hash
(`replicator.rs:1276-1281`), and both the import guard and the type check are
inside `if let Some(ref name)` (`replicator.rs:623`, `replicator.rs:639`). A
topic with no open writer therefore skips both, falls through to
`find_writer_by_hash` finding nothing (`replicator.rs:683`), and is discarded
without being counted as an import rejection.

System topics are metered by per-topic token buckets
(`netfilter.rs:264-286`): e-stop 128 capacity / 64 tokens per second, logs and
presence 64 / 32 (`TokenBucket::new(capacity, refill_per_sec)`,
`netfilter.rs:215`). **E-stop is authenticated before the bucket is charged**
(`replicator.rs:609-617`). Charging first let a forged flood — packets that fail
the MAC and do nothing — drain the bucket so the next genuine halt was dropped
for want of a token, turning a limiter meant to stop an attacker halting the
fleet into one that let an attacker suppress a halt.

The type check compares the local registered type hash against the hash the peer
**announced**, not anything on the packet (`replicator.rs:636-668`). It once
compared a type-name hash against a topic-name hash and therefore rejected every
legitimate import. It catches accidental type mismatch between machines, which
is its stated purpose; it is not a defence against a hostile peer, and the check
that actually stops a wrong-sized write is the payload-size check inside
`ShmRingWriter::write_pod` (`shm_writer.rs:195-197`), reached through the public
`write` (`shm_writer.rs:136-142`).

### Export path

`handle_export` (`replicator.rs:740-896`) selects topics with remote subscribers
that pass `allow_export`, reads each topic's ring under its `ExportSampling`
mode, runs the optimizer chain, then fragments, replicates by reliability tier
and sends per subscriber.

`MAX_EXPORT_BATCH` = 64 bounds one topic's contribution to one tick
(`replicator.rs:49-57`, used at `replicator.rs:766`); the reader resumes from its
own cursor so the remainder goes out next tick rather than being dropped.

Export is driven from the **timer**, not only from the eventfd
(`replicator.rs:905-928`). The only writer of that eventfd is
`TopicRegistry::notify_change`, which fires on register and unregister and
nothing else (`registry.rs:170`, `registry.rs:191`) — so a topic was exported
when it was created and then never again, however much data flowed through it.
Outbound replication stopped as soon as the topology settled, which is the moment
a robot starts doing useful work.

That fix made the tick rate the export rate, so sampling became a per-topic
choice (`shm_reader.rs:26-46`), selected once per topic when the reader is opened
(`replicator.rs:1252-1258`):

| Mode | Config values | Behaviour |
|---|---|---|
| `LatestOnly` (**default**) | `latest`, `latest_only`, `newest`, `sample`, `sampled` | newest slot per tick; the rest counted as skipped |
| `AllSlots` | `all`, `all_slots`, `stream`, `every`, `full` | everything since the last poll, up to the bound |

An unrecognised value is `None` rather than a guess (`shm_reader.rs:48-57`), so
the caller falls through to `export_stream` and then to the default
(`config.rs:337-354`).

`LatestOnly` is the default because it is the bandwidth floor and the safety
heartbeat shares the link — but a decimated topic says so on the console, at most
once per `SKIP_REPORT_INTERVAL` = 10 s (`shm_reader.rs:65-71`,
`replicator.rs:773-781`). From the far end, odometry arriving at 20 Hz because
the exporter sampled it is otherwise indistinguishable from odometry published at
20 Hz.

Two fields the export path hard-codes: `timestamp_ns: 0` on every data message
(`replicator.rs:791`), and the budget argument to `auto_infer` (section 13).

### Transport substitutability

`Transport` is a public trait with five methods (`transport/mod.rs:9-25`), and
`SafetyHeartbeat::tick` and `EstopBroadcaster::broadcast`/`tick` are generic over
it (`heartbeat.rs:223`, `estop.rs:502`, `estop.rs:541`). **The `Replicator` is
not.** It holds a concrete `transport: UdpTransport` field (`replicator.rs:75`),
constructs it directly in `new` (`replicator.rs:131`), and there is no
constructor that takes a transport. `impl Transport for` has six matches in the
crate: `transport/udp.rs:60` is the real one, `estop.rs:654`, `estop.rs:977`,
`estop.rs:997` and `heartbeat.rs:564` are behind `#[cfg(test)]`, and
`heartbeat.rs:349` is a `MockTransport` in `pub mod tests_support`
(`heartbeat.rs:321-323`), which is deliberately *not* cfg-gated so integration
tests can use it — it ships in the library.

So: the trait exists so the heartbeat and e-stop paths can be unit-tested without
a socket, and that is all it does. **Substituting the replicator's transport is
aspirational** and would mean making `Replicator` generic or boxing the field.
Budget for that, not for an afternoon.

### System topics on the replication loop

Three, all defined in `registry.rs:101-103`: `_horus.presence`, `_horus.logs`,
`_horus.estop`. All three go out **framed** through `frame_system_topic`
(`replicator.rs:1131-1177`; presence and logs via `send_system_topic`
`replicator.rs:1180-1199`, e-stop at `replicator.rs:1084`), because
`process_packet` dispatches on `PacketHeader::decode` and an unframed payload is
dropped at the magic check by every peer — which is exactly what was happening to
networked e-stop, while a forged one, framed correctly by the attacker,
propagated fine.

The framer's sequence must advance per message (`replicator.rs:1144-1155`). A
fixed 0 delivered the first e-stop episode and silently dropped every later one,
because framed system messages take the normal data path and dedup runs before
the system-topic dispatch. Presence and log replication had the same bug for the
same reason (`replicator.rs:1181-1186`).

E-stop delivery is triple-redundant: multicast, unicast to every known peer, and
`RETRY_COUNT` = 3 retries at `RETRY_INTERVAL` = 10 ms
(`estop.rs:4-7`, `estop.rs:470-471`, `estop.rs:502-538`). The retries are
*scheduled* at 10, 20 and 30 ms (`estop.rs:530-536`) but drained by
`EstopBroadcaster::tick` from the 50 ms replicator timer
(`replicator.rs:1051-1053`), so in practice they land on the following ticks
rather than at those offsets. Its body is
`[host_id_hash:u16][reason_len:u16][reason][timestamp_ns:u64]` with a 32-byte
HMAC-SHA256 tag appended when `HORUS_ESTOP_KEY` is set; the tag covers
`b"horus.estop.v1" || body`, so `reason_len` — which decides where the body ends
— is itself authenticated (`estop.rs:40-52`, `estop.rs:86-104`,
`mac.rs:40`). The reason is capped at 200 bytes (`estop.rs:66`) and the freshness
window is ±5 s (`estop.rs:26-27`). Sends are best-effort but **not silent**: a
broadcast that reached nobody prints a CRITICAL line
(`replicator.rs:1099-1106`, `replicator.rs:1054-1059`).

**With no `HORUS_ESTOP_KEY`, remote e-stop does not work at all.**
`estop_packet_is_authentic` returns false when no key is provisioned
(`estop.rs:157-165`), and `process_packet` drops the packet before the bucket and
before the handler (`replicator.rs:609-612`). The HMAC is not an optional
hardening step on this path; it is the whole admission criterion.

Known limitation, stated in the source: a node joining an already-e-stopped fleet
does not learn of it — there is no join-time state sync
(`replicator.rs:1069-1071`).

---

## Section 21 — Phase 1 success criteria

`tests/phase1_validation.rs` cites this section as the source of its success
criteria (`tests/phase1_validation.rs:2`). The criteria themselves were never
written down anywhere in this repository, so what follows is not the original
list. It is **what the test file actually asserts**, read out of the test bodies
rather than out of the section banners, because on two of them the banner and the
code disagree.

| SC | What the test asserts | Test |
|---|---|---|
| 1 | a `test_config(0)` replicator starts and `is_running()` is true | `tests/phase1_validation.rs:22` |
| 3 | 1000 `encode_single` + `decode_packet` round trips average under 1 ms each | `tests/phase1_validation.rs:33` |
| 4 | a replicator with no peers is still running after 100 ms. It does **not** measure CPU — the "~0% CPU" claim is in the comment at `:64`, not in an assertion | `tests/phase1_validation.rs:63` |
| 5 | nothing; the body is empty and compilation is the test | `tests/phase1_validation.rs:76` |
| 8 | a latch opens and an ack clears it, `auto_infer("robot.estop", …)` is `Immediate`, and `default_for(Immediate)` is `Latched`. It does **not** assert the copy count | `tests/phase1_validation.rs:86` |
| 9 | with `imu` registered as `Publisher`, `allow_import` denies both `imu` and an unknown topic | `tests/phase1_validation.rs:116` |
| 10 | a 10 ms-interval heartbeat with threshold 3, given one received beat then 60 ms of silence, reports exactly one `SafeState`. The banner says 150 ms and the function name says 200 ms; neither number appears in the test | `tests/phase1_validation.rs:128` |
| 11 | a 1 MB message fragments and reassembles to the same length | `tests/phase1_validation.rs:155` |
| 12 | `check_encoding(native_pod())` is `SameEndian` | `tests/phase1_validation.rs:184` |
| 13 | an f64 survives a byte-swap round trip | `tests/phase1_validation.rs:192` |
| 15 | setting `config.peers` yields `DiscoveryMode::Unicast` on port 9100. It sets the field directly and never reads `HORUS_NET_PEER` | `tests/phase1_validation.rs:212` |

**SC2, SC6, SC7 and SC14 have no test and no recoverable text.** Four numbered
criteria are absent from the file — there is no marker of any kind for them — and
nothing in the repository or its history records what they were. Do not infer
them. If you know what they were, add them here with their tests; until then this
table is the whole of section 21 that can be honestly stated.

SC5's claim is worth reading against this crate's own `Cargo.toml`: the runtime
dependencies are `horus_core`, `horus_sys`, `memmap2` and, on Linux and macOS,
`libc` (`horus_net/Cargo.toml:9-19`). `serde`, `bincode`, `horus_types` and
`horus-robotics` are dev-dependencies only (`horus_net/Cargo.toml:21-25`), so the
claim holds for anything that ships.

---

## Configuration defaults

These are the values `NetConfig` produces with no environment set. Every row
marked ✓ is asserted by `config_defaults_match_blueprint`
(`tests/safety_tests.rs:149-160`); change a value here and that test must change
with it.

| Field | Default | Env override | Source | Asserted |
|---|---|---|---|---|
| `enabled` | `true` | `HORUS_NO_NETWORK=1` | `config.rs:228-233` | ✓ |
| `port` | `9100` | `HORUS_NET_PORT` | `config.rs:234-237` | ✓ |
| `multicast_group` | `224.0.69.72` | `HORUS_NET_MULTICAST` | `config.rs:238-239` | |
| `peers` | `[]` (empty) | `HORUS_NET_PEER` (csv) | `config.rs:240-247` | |
| `secret` | none | `HORUS_NET_SECRET` | `config.rs:248` | |
| `import` | `auto` | `HORUS_NET_IMPORT` | `config.rs:255`, `config.rs:95-96` | ✓ |
| `deny_export` | `[]` (empty) | `HORUS_NET_DENY_EXPORT` (csv) | `config.rs:256` | ✓ |
| `export_stream` | `[]` (empty) | `HORUS_NET_EXPORT_STREAM` (csv) | `config.rs:257` | |
| `safety.heartbeat_ms` | `50` (200 on WSL2) | `HORUS_NET_HEARTBEAT_MS` | `config.rs:174-177` | ✓ |
| `safety.missed_threshold` | `3` (5 on WSL2) | `HORUS_NET_MISSED_THRESHOLD` | `config.rs:179-182` | ✓ |
| `safety.on_link_lost` | `warn` | `HORUS_NET_ON_LINK_LOST` | `config.rs:188-192` | ✓ |
| `estop_remote` | `warn` | `HORUS_ESTOP_REMOTE` (`off` to ignore) | `config.rs:259`, `config.rs:64-74` | |
| `optimizers` | `[]` (empty) | `HORUS_NET_OPTIMIZERS` (csv) | `config.rs:260` | ✓ |
| `topic_overrides` | `[]` (empty) | — (no path; see section 13) | `config.rs:261` | |

Related defaults that are not `NetConfig` fields: the inbound source filter is
private/loopback/link-local space, overridable with `HORUS_NET_ALLOW_PEERS`
(`netfilter.rs:90-102`, `netfilter.rs:132-137`); peer discovery misses is 3
native and 6 on WSL2, overridable with `HORUS_NET_DISCOVERY_MISSES`
(`peer.rs:70-73`); log replication level is `Warning`, overridable with
`HORUS_NET_LOG_LEVEL` (`log_replication.rs:11-36`); and the e-stop HMAC key comes
from `HORUS_ESTOP_KEY` (`mac.rs`, and see section 18).

Two things a contributor should know about that assertion test:

- It calls `NetConfig::test_config(9100)`, whose doc comment says "no env var
  reads" (`config.rs:295`) — but its `safety` field is `SafetyConfig::default()`
  (`config.rs:306`), which reads three environment variables and probes
  `/proc/version` for WSL2 (`config.rs:160-192`). **The test therefore fails on a
  WSL2 machine, or with `HORUS_NET_HEARTBEAT_MS` set in the shell.** It is not
  hermetic. Fixing it means giving `test_config` a literal `SafetyConfig`.
- The three env-derived list fields — `import`, `deny_export`, `optimizers` —
  were once hard-coded with no env read at all, which is why the entire
  `[network]` section of `horus.toml` was parsed and then silently discarded, and
  `ImportMode::Deny` and `AllowList` were unreachable in shipped code
  (`config.rs:249-254`).

### How `horus.toml` reaches `horus_net`

Only through environment variables that `horus run` sets before spawning the
process (`horus_manager/src/commands/run/mod.rs:79-142`). There is no config file
read anywhere in `horus_net`. `apply_network_config` sets eight variables —
`HORUS_NO_NETWORK`, `HORUS_NET_SECRET`, `HORUS_NET_DENY_EXPORT`,
`HORUS_NET_OPTIMIZERS`, `HORUS_NET_IMPORT`, `HORUS_NET_HEARTBEAT_MS`,
`HORUS_NET_MISSED_THRESHOLD`, `HORUS_NET_ON_LINK_LOST` — and an environment
variable the operator has already set wins over the file
(`horus_manager/src/commands/run/mod.rs:76-78`, `:91-95`).

The `[network]` table accepts exactly six keys: `enabled`, `import`,
`deny_export`, `secret`, `optimize`, and a `safety` sub-table of `heartbeat_ms`,
`missed_threshold` and `on_link_lost` (`horus_manager/src/manifest.rs:276-315`).
Everything else `horus_net` reads is environment-only and has no `horus.toml`
spelling: `HORUS_NET_PORT`, `HORUS_NET_MULTICAST`, `HORUS_NET_PEER`,
`HORUS_NET_EXPORT_STREAM`, `HORUS_NET_ALLOW_PEERS`,
`HORUS_NET_DISCOVERY_MISSES`, `HORUS_NET_LOG_LEVEL`, `HORUS_ESTOP_REMOTE` and
`HORUS_ESTOP_KEY`.

If you add a config field, add the env read *and* the `apply_network_config` arm,
or the key is inert.

## Where these rules are actually enforced

Two workflows, and they cover different halves of this file. Check which one your
test lands in before you assume it runs.

- **Unit tests inside `src/`** run in `ci.yml`'s test job:
  `cargo test --workspace --exclude horus_py --lib --no-fail-fast`
  (`.github/workflows/ci.yml:203`). `--lib` reaches
  `default_reliability_matches_blueprint` (`reliability.rs:189`), `total_overhead`
  (`wire.rs:660`) and the optimizer, netfilter and config unit tests. It does not
  reach anything in a `tests/` directory.
- **Integration tests under `tests/`** run in `integration-tests.yml`:
  `cargo test --workspace --exclude horus_py --release --test '*'`
  (`.github/workflows/integration-tests.yml:127`), on pull requests to `main` and
  `dev`. That is what runs `phase1_validation.rs` (section 21) and
  `config_defaults_match_blueprint` in `safety_tests.rs`.
- **`horus_net` is excluded from the Windows job** entirely
  (`.github/workflows/multi-platform.yml:140-150`), and cross-compiled for
  `armv7-unknown-linux-gnueabihf` in the 32-bit job
  (`.github/workflows/multi-platform.yml:275-329`) — which is what makes the
  32-bit `usize` reasoning in section 6 a live concern rather than a hypothetical.

There is currently **no checker that this file and the citations agree**. Nothing
scans `horus_net/src` for `blueprint section N` and asserts a matching heading
here. Until one exists, the last line of "Changing something in here" is a
convention, not a gate.

If you write that checker: every cited section has a heading of the exact form
`## Section N — Title`, but match on a word boundary, not on `contains("## Section
N")` — a substring test for section 1 matches the heading for section 18, and a
substring test for section 2 matches nothing at all even though section 21
exists. Anchor on the trailing space or em-dash. It also needs to know that the
gaps are deliberate: sections 1–5, 16, 19 and 20 have no heading here on purpose,
and a checker that demands 1..=21 will fail forever.

## Stale comments in the source

Three source comments contradict the code they sit on. They are listed together
so a reader who trips over one knows it is a known class, and so a future fix can
clear them in one pass. In every case the code is the specification.

| Comment | Says | Actually |
|---|---|---|
| `guard.rs:3-4` | "Import default: deny all" | the default is `Auto` (`config.rs:95-96`) — section 9 |
| `wire.rs:159` | `topic_hash` is "FNV-1a hash of the topic type name" | it is the hash of the **topic name** (`replicator.rs:767`, `replicator.rs:1140`) — section 6 |
| `discovery.rs:55` | "Wire-format topic entry (72 bytes)" | `WireTopicEntry::SIZE` is 76 (`discovery.rs:70`) — section 17 |

`flow_control.rs:4` ("Phase 1: should_send() always returns true") belongs on
that list too; it is called out in section 14 where it does the most damage.

## Sections that nothing cites

Sections 1–5, 16, 19 and 20 are cited by no file in `horus_net/src` or
`horus_net/tests`, and no text for them survives anywhere in the repository or
its history (see "What was searched for" at the top). They are deliberately
absent here rather than invented. If you are restoring the original blueprint and
know what they covered, add them with their citations; do not renumber the
sections above, because the numbers are load-bearing — they are what the source
comments point at.

Separately, five source comments use a `§N` notation that is **not** this
numbering: `§1` for the bit-7 discriminator (`replicator.rs:356`,
`replicator.rs:1346`, `discovery.rs:622`), `§3` for the reassembly DoS bounds
(`fragment.rs:497`) and `§2/§6` for the `process_packet` dispatch
(`replicator.rs:1335`). Those numbers do not line up with the blueprint sections
— §1 here would be an uncited section, and blueprint §3 is not fragmentation.
**Where they come from is unverified**: nothing in this repository defines that
numbering, and the guess that it is a security review's own scheme is a guess. Do
not treat those `§` marks as references into this file. (Two further `§` hits,
`mac.rs:197` and `mac.rs:589`, are ordinary RFC section references and are
unrelated.)

## What this file does not cover

Modules with no blueprint citation, and therefore no section here:
`estop.rs` (beyond what section 18 needs), `mac.rs`, `netfilter.rs`, `peer.rs`,
`presence.rs`, `log_replication.rs`, `registry.rs`, `shm_reader.rs`,
`shm_writer.rs`, `event_loop/`, `transport/`. Their behaviour is described in
their own module docs and, where a rule of theirs is load-bearing for a cited
section, it is quoted above with its file and line. In particular the SHM ring
format that section 18's export path reads is documented with `shm_reader.rs` and
`shm_writer.rs`, not here.

This file also says nothing about the crate's trust model beyond what a section
needs. Read `lib.rs:14-44` before deploying: topic replication is
**unauthenticated** — no MAC, no handshake, no source binding on the data path —
so any host inside the allowed peer ranges can write arbitrary bytes into
importable local SHM topics, actuation commands included, and can replay anything
it captured. Only `_horus.estop` is authenticated, and only when
`HORUS_ESTOP_KEY` is set. Closing that is a wire-format change and is not
implemented.

## Changing something in here

- **Wire layout (section 6 or 17).** Every field offset is hand-written in an
  `encode`/`decode` pair and there are no compile-time layout assertions in this
  crate — grepping for `static_assert`, `const _:` and `size_of::<PacketHeader>`
  across `horus_net` returns nothing. Change both halves, update the offset table
  above, and consider whether `VERSION` must move — `PacketHeader::decode` and
  `AnnouncementHeader::decode` both reject a mismatched version outright
  (`wire.rs:77-80`, `discovery.rs:178-181`), so a version bump is a hard
  fleet-wide cut, not a negotiation.
- **A new packet kind.** Take a free `PacketFlags` bit — only bit 6 is free, and
  bit 7 is spoken for. Add the branch to `process_packet`'s dispatch table
  (`replicator.rs:354-524`) and to the table in section 18.
- **A new optimizer.** Edit the match at `optimize/mod.rs:55-75`. There is no
  registry. Add a row to the table in section 12 and say whether it is normative
  or aspirational.
- **A new transport.** Implement `Transport` (`transport/mod.rs:9-25`), then
  make `Replicator` generic or box its field (`replicator.rs:75`,
  `replicator.rs:131`) — the trait alone does not make it substitutable.
- **A config default.** Change `config.rs`, the table above, and
  `config_defaults_match_blueprint` (`tests/safety_tests.rs:149-160`) together.
  Add the env read and the `apply_network_config` arm or the key is inert.
- **A new `blueprint section N` citation.** Add the section here first. A
  citation with no section is the failure this file exists to end — and, until
  the checker described under "Where these rules are actually enforced" is
  written, nothing but review will catch you.
