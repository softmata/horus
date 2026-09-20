# horus_core

The runtime: nodes, the scheduler and its four executors, shared-memory topics,
the tensor pool, actions, services, errors. 112,344 lines under `src/`
(`find horus_core/src -name '*.rs' | xargs wc -l`), of which 14,340 are `///`
doc lines and 2,280 are `//!` module prose.

Almost none of that prose is reachable through `cargo doc`. This file is the
index to it. It is written for someone changing this crate, not for someone
using it.

**If you are writing a robot, you want the `horus` crate**, whose rustdoc is
public and whose `prelude` is the supported surface (`horus/src/lib.rs:581`).
The user-facing API reference lives at `content/docs/rust/api/core.mdx` in
`horus-docs` — 1,234 lines of hand-written MDX, not generated from this source.
See [Who actually depends on this crate](#who-actually-depends-on-this-crate)
for what "internal" does and does not mean in practice.

---

## Why cargo doc shows you nothing

`horus_core/src/lib.rs` is 96 lines. Eleven of its twelve `pub mod`
declarations carry `#[doc(hidden)]` — ten unconditionally, and `testing` both
hidden and `cfg`-gated:

| Module | Hidden? | Declared at |
|---|---|---|
| `actions` | yes | `lib.rs:20-21` |
| `communication` | yes | `lib.rs:22-23` |
| `core` | yes | `lib.rs:24-25` |
| `drivers` | yes | `lib.rs:26-27` |
| `error` | yes | `lib.rs:28-29` |
| `memory` | yes | `lib.rs:30-31` |
| `params` | yes | `lib.rs:32-33` |
| `scheduling` | yes | `lib.rs:34-35` |
| `services` | yes | `lib.rs:36-37` |
| `terminal` | **no** | `lib.rs:38-44` |
| `types` | yes | `lib.rs:45-46` |
| `testing` | yes, and `cfg(any(test, feature = "test-utils"))` | `lib.rs:50-52` |

There is one more declaration, `pub(crate) mod utils;` at `lib.rs:47`, which is
private and so never rendered either.

Every crate-root re-export is hidden as well, apart from one: `ControlCommand`
(`lib.rs:79`). `Topic`, `Scheduler`, `Node`, `HorusError` and the rest go out
behind `#[doc(hidden)]` at `lib.rs:56-86`, and the four re-exported
dependencies (`bytemuck`, `paste`, `serde_json`, `serde_yaml`) at
`lib.rs:88-96`.

`#[doc(hidden)]` on a module strips the module *and everything inside it* from
the rendered output. A public re-export *out of* a hidden module still renders,
which is why `ControlCommand` survives, and a `#[macro_export]` macro declared
inside a hidden module also renders, because `macro_export` puts it at the
crate root regardless. Nothing else in this crate takes either route.

So the crate's front page carries exactly four things. This is observed, not
inferred — `cargo doc -p horus_core --no-deps` with `CARGO_TARGET_DIR` pointed
at a scratch directory produces a `sidebar-items.js` containing precisely:

```js
window.SIDEBAR_ITEMS = {"enum":["ControlCommand"],"macro":[["action",1],
["hlog",1],["hlog_every",1],["hlog_once",1],["horus_internal",1],
["impl_frame_id_field",1],["impl_tensor_accessors",1],["impl_tensor_backed",1],
["impl_timestamp_field",1],["message",1],["register_driver",1],["service",1],
["standard_action",1],["topics",1]],"mod":["terminal"]};
```

That is the sixteen lines of crate prose at `lib.rs:1-16`, one module, one
enum, and fourteen macros. Fifteen `#[macro_export]` macros exist in the crate;
the fifteenth, `__hlog_level_const`, carries its own `#[doc(hidden)]`
(`core/hlog.rs:174-175`) and is the one missing from that list. Everything
else — 112,000 lines and the arguments in them — renders nowhere.

Two consequences worth stating plainly.

**The prose is lint-gated but not published.** The `doc` job at
`.github/workflows/ci.yml:308` runs
`cargo doc --workspace --no-deps --exclude horus_manager` (`ci.yml:345`) with
`RUSTDOCFLAGS: -D warnings`, but only when the ref is `main` or the base ref is
`main` (`ci.yml:347`) — so a PR into `main` is gated and a PR into `dev` is
not. Rustdoc's lints fire inside `#[doc(hidden)]` items: a broken intra-doc
link in a hidden module is a warning by default and a hard error under
`-D warnings`, verified on a two-line probe crate. So every one of those 14,340
doc lines is held to a standard, and then discarded. No workflow publishes
`target/doc` (`grep -rn 'cargo doc\|target/doc\|gh-pages\|deploy-pages'
.github/workflows/` matches only the comment at `ci.yml:338` and the command at
`ci.yml:345`). `docs.horusrobotics.dev`, declared as this crate's
`documentation` at `Cargo.toml:11`, is the `horus-docs` Next.js site
(`horus-docs/app/sitemap.ts:7`), not rustdoc output. And docs.rs cannot help,
because none of these crates are on crates.io (`README.md:70-73`).

**Instructions that users must follow are invisible.** The clearest case:
`memory/rt_allocator.rs:15-19` tells you to write

```rust
#[global_allocator]
static ALLOC: horus_core::memory::rt_allocator::RtAwareAllocator =
    horus_core::memory::rt_allocator::RtAwareAllocator;
```

in your own `main.rs`, because without it `.no_alloc()` is inert
(`rt_allocator.rs:28-38`). `memory` is hidden here and hidden again in the
facade (`horus/src/lib.rs:389-390`), so that page renders in neither crate. The
snippet is only correct at all because
`horus_core/tests/rt_alloc_guard_installed.rs:39-41` compiles it as that test
binary's real global allocator — an earlier version of the same paragraph named
`horus_core::memory::RtAwareAllocator`, a path that does not exist, and nothing
caught it (`rt_allocator.rs:21-26`, `rt_alloc_guard_installed.rs:35-38`).

## How to read the internals

`--document-private-items` is **not** enough on its own. It disables rustdoc's
private-stripping pass, not its hidden-stripping pass, so the hidden modules
stay stripped. Use the `internal-docs` feature, which lifts the
`#[doc(hidden)]` attributes and works on stable:

```sh
cargo doc -p horus_core --no-deps --document-private-items \
  --features internal-docs --open
```

The nightly form below still works and needs no feature, but it requires a
nightly toolchain and an unstable flag, which is why the feature exists:

```sh
RUSTDOCFLAGS="-Z unstable-options --document-hidden-items" \
  cargo +nightly doc -p horus_core --no-deps --document-private-items --open
```

Both halves matter. `--document-hidden-items` restores the hidden modules;
`--document-private-items` restores the `pub(crate)` ones — `topic::header`
(`communication/topic/mod.rs:118`), `topic::dispatch` (`:131`),
`topic::seqlock` (`:134`), `scheduling::rt_executor`
(`scheduling/mod.rs:34`), `scheduling::primitives` (`:31`) and their
neighbours — which is where most of the protocol writing actually lives.

Each half was checked separately on a probe crate rather than taken from
memory, because this is the one instruction in this file a reader will act on:
plain `rustdoc --document-private-items` leaves a `#[doc(hidden)] pub mod`
absent from the sidebar and adds only the `pub(crate)` module; passing
`--document-hidden-items` without `-Z unstable-options` fails with *"the
`-Z unstable-options` flag must also be passed"*; the command above, run
end-to-end through cargo, lists the hidden module. If you doubt any of it, a
two-line `lib.rs` reproduces all three in ten seconds.

`horus doc` does not do this. It documents the *user's* project, from that
project's directory, pointing cargo at `.horus/Cargo.toml`
(`horus_manager/src/commands/doc.rs:1-4` and `:16-48`).

## The module index

Sizes are `src/` line counts. "Read first" names the file whose header carries
the real argument, not the file with the most code.

| Module | Lines | Read first |
|---|---:|---|
| `scheduling` | 39,523 | `scheduling/node_builder.rs:6-37` |
| `communication` | 31,649 | `communication/topic/mod.rs:1-112` |
| `memory` | 11,165 | `memory/rt_allocator.rs:1-88` |
| `core` | 8,918 | `core/node.rs:953-971` |
| `actions` | 5,925 | `actions/mod.rs:1-122` |
| `types` | 4,405 | `types/mod.rs:1-22` |
| `error.rs` | 2,937 | `error.rs:1-64` |
| `services` | 2,492 | `services/mod.rs:1-60` |
| `drivers` | 1,908 | `drivers/mod.rs:1-19` |
| `params.rs` | 1,666 | `params.rs:1-3` |
| `testing` | 1,449 | gated on `cfg(test)` or `feature = "test-utils"` (`Cargo.toml:91-92`) |
| `terminal.rs` | 63 | `terminal.rs:1-19` |

Those twelve plus `utils.rs` (148) and `lib.rs` (96) are the whole crate.

### communication

`communication/topic/mod.rs:1-112` is the single most important header in the
crate: the backend detection matrix, the thread-ownership contract, the
lock-free ring protocols, the `read_latest` use-after-free argument and the
drop-safety rules. Read it before touching anything under
`communication/topic/`. Its closing "Verification" paragraph (`:106-112`) is
stale and should not be trusted: it claims "16 loom exhaustive concurrency
tests … for SPSC, SPMC, MPSC, and MPMC algorithms, including `read_latest` +
`try_recv` races", where the tree holds ten loom targets with 36 `#[test]`
functions between them, none of which mentions `read_latest`. The models that
covered those four ring algorithms were `loom_ring_buffers`, deleted with the
intra backends (`ci.yml:379-380`).

Then, in order of how much they will surprise you:

- `communication/topic/dispatch.rs:2-118` — one complete send/recv path per
  backend, plus a measured account of where the remaining ~55 ns against a raw
  ring goes. The ablation table at `:31-39` is the useful part: removing the
  consumer's head gate makes things 2-4 ns *worse*. Its conclusion (`:54-61`)
  is that anyone trying to close the gap should change the protocol, not delete
  work from these functions, and should check first whether 55 ns of a 1 kHz
  period is worth anything.
- `communication/topic/shm_layout.rs:1-50` — the authoritative byte layout,
  and why it is a module rather than a comment. `horus_net` once copied a table
  of offsets and the copy rotted in four independent places (`:11-16`) while
  feeding an IPC region from unauthenticated network data. Each constant is now
  tied to its field by `offset_of!` in a `const` block (`:20-24`), so a
  reordered `TopicHeader` field is a build failure.
- `communication/topic/header.rs:24-66` — `TOPIC_VERSION`, currently `4`
  (`:66`), and why v4 is a hard break rather than a soft migration. Neither of
  its two changes (monotonic timestamps, `layout_kind`) is detectable in-band
  by a v3 reader, so the open path refuses the segment instead.
- `communication/topic/seqlock.rs:1-47` — the drop-oldest fanout ring's
  per-slot version protocol.
- `communication/pod.rs:1-77` — POD auto-detection by `!needs_drop::<T>()`,
  and the caveat at `:10-26` that it is a heuristic and unsound for `bool`,
  fieldless enums and `NonZero*`. Prefer explicitly encoded integer fields in
  messages you intend to send as fixed-size.
- `communication/topic/migration.rs:2-5` and `communication/topic/registry.rs:1-5`
  — CAS-locked backend migration, and the process-local epoch counter that lets
  sibling `Topic` instances notice it with a ~1 ns L1 read instead of a ~20 ns
  SHM read.

### scheduling

`scheduling/node_builder.rs:6-37` is the entry point: the decision tree and
table for the five execution classes (`Rt`, `Compute`, `Event(String)`,
`AsyncIo`, `BestEffort` — `scheduling/types.rs:279-291`), and the rules that
catch people out (`:33-37`). One class per node, last call wins with a warning;
`.rate()` on a `BestEffort` node auto-promotes it to `Rt`; `.compute().rate()`
stays `Compute` and the rate becomes informational; a chain without `.build()?`
is silently dropped.

The executors each carry an ASCII architecture diagram in their header:
`scheduling/rt_executor.rs:8-22`, `compute_executor.rs:7-22`,
`event_executor.rs:7-21`, `async_executor.rs:7-22`. Read
`rt_executor.rs:366-429` for the cyclic-wait rewrite — an absolute-deadline
`clock_nanosleep` with a bounded guard spin, replacing a loop that both drifted
(`:382-389`) and busy-waited hard enough to be dequeued by Linux RT bandwidth
control for roughly 50 ms once a second (`:391-401`), a tail that only appears
once RT priority is actually granted (`:403-408`). The trade it makes — median
wake jitter regressing from the old spin's ~100 ns to hrtimer precision, bought
to delete that ~50 ms tail — is stated at `:416-423` rather than buried, along
with the two escape hatches: `HORUS_RT_WAIT=spin` restores a pure spin and
`HORUS_RT_SPIN_GUARD_US` retunes the guard (read at `:675` and `:684`, the
env-var name held at `:472`). The constants that bound it are at `:431-467`:
`SPIN_GUARD_DEFAULT_NS = 20_000` (`:445`), `SPIN_GUARD_PERIOD_SHIFT = 4`
(`:456`), `MIN_SLEEP_SLACK_NS = 10_000` (`:467`).

`scheduling/primitives.rs:1-24` explains why the RT-side reporting path uses
`rt_diag` and not `print_line`: `print_line` does `isatty` + `tcgetattr`, takes
the process-global stdout lock, and can then block for as long as a slow
console takes — on the thread driving an actuator.

### memory

`memory/rt_allocator.rs:1-88` is the `.no_alloc()` contract, and it is the
best-written safety argument in the crate. The parts to not miss: the
installation step is downstream opt-in and a library cannot do it for you
(`:9-26`); without it `.no_alloc()` is silently unenforced, and `is_installed`
/ `warn_if_unenforced` exist so that downgrade can be reported (`:28-38`); a
violation panics inside the tick and routes through the node's `FailurePolicy`
rather than aborting (`:40-66`); and the caveat at `:68-74` that the default
`failure_policy` is `None`, which `apply_failure_policy_after_panic` treats as
log-and-continue — so a `.no_alloc()` node with no explicit policy keeps
ticking and keeps violating, detected and counted, but not acted on.

Elsewhere: `memory/tensor_pool.rs:2-60` for the cross-process zero-copy pool
layout; `memory/backend.rs:1-21` for the `PoolBackend` trait that controls
where tensor bytes are allocated — `MmapBackend` is the only built-in (`:10`,
implemented at `:396`), the three CUDA backends at `:12-16` are future work,
and the `HeapBackend` at `:570` is test-only; and `memory/simd.rs:1-40` for why
the hand-written AVX2 copy paths were deleted — both directions lost to
`std::ptr::copy_nonoverlapping`, with the measured table at `:19-23`.

### core

`core/node.rs:953-971` carries the `Node` trait (`pub trait Node: Send` at
`:971`). Only `tick()` is required (`:992`); `name()` defaults to the type name
(`:976-982`), `init()` and `shutdown()` default to `Ok(())` and return
`crate::error::HorusResult<()>` (`:987-989`, `:997-999`).
`on_parameter_change` is documented at `:1030-1035` as **not wired** — the
scheduler does not call it, and overriding it today has no effect; register a
callback via `RuntimeParams::on_change()` instead.

`core/clock.rs:1-13` has the clock-backend table (`WallClock`, `SimClock`,
`ReplayClock`); `core/tick_context.rs:1-8` explains the thread-local ambient
context that `horus::now()`, `dt()` and `rng()` read; `core/log_buffer.rs:170`
onwards documents the per-slot seqlock in the SHM log ring, and `:79` plus
`:477-494` document how a writer that died mid-write is detected, skipped, and
its seqlock reset to even so the slot can be reclaimed.

### actions, services, errors, drivers, params, types, terminal

- `actions/mod.rs:1-122` — the goal/feedback/result pattern (`:13-18`), server
  and client builders, preemption policies (`:108-110`). At 122 lines it is the
  longest module header in the crate, just ahead of `dispatch.rs` (117) and
  `topic/mod.rs` (112).
- `services/mod.rs:1-60` — request/response over two topics per service
  (`{name}.request`, `{name}.response`, `:41-43`), request-ID filtering for
  concurrent clients (`:47-49`), and a ROS2 equivalence table (`:51-60`).
- `error.rs:1-64` — `HorusError` as an umbrella over eleven structured domain
  sub-errors, with the variant table at `:10-22` and the matching idiom at
  `:24-45`.
- `drivers/mod.rs:1-19` — loading nodes from `horus.toml` `[hardware]`. The
  reserved keys that are consumed rather than passed through as `NodeParams`
  are the literal list at `drivers/mod.rs:111-124`; the `exec:` prefix dispatch
  to `ExecDriver::from_config` is at `:217-229`.
- `params.rs` — runtime key-value parameters, with the eight `ValidationRule`
  variants at `:13-32`. Its header is three lines; the module is 1,666 and
  undocumented by comparison.
- `types/mod.rs` — zero-copy tensor descriptors and element types. Its header
  is a single line, and it hides eight of its own eleven children again at
  `:3-22` (`action_chunk`, `image_encoding` and `point` are the three that
  would render if the parent did).
- `terminal.rs:1-19` — the only module that renders today, and it renders
  because it must: `println!`/`eprintln!` panic when the write fails, and a
  robot's stdout fails routinely. Anything printing from a node, a background
  thread or a safety path uses these.

## Load-bearing contracts

Four things you can break without the compiler noticing.

**`Topic<T>` is `Send` and deliberately not `Sync`.** `unsafe impl<T:
TopicMessage> Send for Topic<T> where T::Wire: Send` at
`communication/topic/mod.rs:3711`; there is no `Sync` impl, and the comment at
`:3713-3719` says why — `send`/`recv` take `&self` and mutate
`registered_pub`, `registered_sub`, `owner_attempts` and a `RefCell` keep-alive
queue, so two threads sharing one `&Topic<Image>` can release the same pool
slot twice, freeing shared memory that subscribers are still reading. To share
one handle across threads you need a `Mutex<Topic<T>>`. **An `RwLock` is not
enough**: a read guard hands out concurrent `&Topic`. Otherwise clone per
thread — `Clone` gives the new thread its own independent state. The module
header states both at `communication/topic/mod.rs:36-48`, and it is the
paragraph to keep in step if the impls change. `RingTopic<T>` is `Send` for
`T: Send` on the same reasoning (`:777`), also with no `Sync`. A compile-time
guard at `:3721-3728` asserts `Topic<u64>: Send` so that losing `Send` fails
here rather than at a `thread::spawn` in a downstream crate. This is the
current state of a contract that was previously wrong in both directions:
hand-written `unsafe impl Sync` blocks existed while the prose claimed
`!Send + !Sync`, so safe code could race the unsynchronised state with no
`unsafe` of its own (`:42-48`, `:779-793`).

**The ring protocols differ per backend, and the ordering is the proof.**
`communication/topic/mod.rs:63-75`: SPSC needs no CAS (producer `Release`s
head, consumer `Acquire`s it); SPMC has consumers CAS the tail with `AcqRel` to
claim exclusive read slots; MPSC has producers CAS head with per-slot
Lamport-style sequence numbers; MPMC CASes both ends. The seqlock fanout ring
is separate again: an odd stamp `(pos << 1) | 1` means a write is in progress
and an even stamp `pos << 1` means done, the producer sets odd *before*
touching data and even *after*, and the consumer must see exactly `next << 1`
on both sides of its copy or discard it (`seqlock.rs:19-34`). Publication
order — done-stamp `Release`, then head `Release` — is what makes `head > next`
imply the done stamp for `next` is visible (`seqlock.rs:36-38`). Retries are
bounded at `SEQLOCK_MAX_ATTEMPTS = 16` (`seqlock.rs:55`).

Ten loom models live in `horus_core/tests/loom_*.rs` and are the executable
form of these arguments — 36 `#[test]` functions between them. Six have a named
step in ci.yml's `loom` job (`ci.yml:349`): `loom_fanout` (`:383`),
`loom_pod_broadcast` (`:392`), `loom_concurrency` (`:397`), `loom_migration`
(`:402`), `loom_migration_data_plane` (`:414`), `loom_participant` (`:419`).
The other four — `loom_mp_claim`, `loom_sp_mp_flag`, `loom_spmc_epoch_flush`,
`loom_tensor_pool` — are named by no workflow, but they are **not** unrun:
`integration-tests.yml:127` runs
`cargo test --workspace --exclude horus_py --release --test '*'`, and that glob
matches every integration target, loom included. So they do execute on every PR
into `main` or `dev` — in release, at `--test-threads=1`, inside a 45-minute
sweep whose failure reads as an integration break rather than a protocol break,
and with none of the `RUST_LOG=loom=debug` the named steps set. If you change a
protocol one of those four covers, run it yourself and read the output:

```sh
cargo test -p horus_core --test loom_mp_claim -- --nocapture
```

One test is genuinely run nowhere by default:
`loom_pod_broadcast.rs:224` carries
`#[ignore = "demonstrates the OLD protocol failing; run by hand to validate the
model"]`. That is deliberate — it is the gate that proves the model is not
vacuous, and it is meant to be run by hand when you change the protocol it
models.

**`read_latest()` requires `T: Copy` on multi-consumer backends.**
`communication/topic/mod.rs:77-97` sets out the TOCTOU: `read_latest` computes
a slot index, a consumer CAS-advances tail past that slot and
`assume_init_read`s the value out, `Drop` frees the heap allocation, and
`read_latest` then reads freed memory. `Copy` types have no `Drop` and no heap
pointers, so the bytes are always safe to bitwise-copy. Single-consumer
backends keep `T: Clone` (`:93-94`); the `Topic`-level `read_latest` requires
`Copy` unconditionally because migration can switch the backend from SPSC to
SPMC at runtime (`:96-97`).

**The executor model gives you no ordering between RT nodes.** The comment at
`scheduling/scheduler/mod.rs:3097-3117` is the authority, and `rt_chains` at
`:3118-3119` is `groups.rt_nodes.into_iter().map(|n| vec![n]).collect()` — one
OS thread per RT node, always, so a stalled node cannot block its RT siblings.
The block also records that the previous claim (dependent RT nodes grouped into
chains and ticked sequentially) was never true: `independent_chains` read no
edges and was queried against the wrong graph (`:3104-3111`). Stated at
`:3113-3117`: an RT consumer may read its RT producer's previous-tick value.
Nodes that need ordering must share an execution class and be sequenced by the
main loop, or synchronise through the data they exchange.

Note that `rt_executor.rs:2-22` still describes the old model ("Runs all RT
nodes on an isolated OS thread … ticks RT nodes sequentially in priority
order"), and its diagram draws three RT nodes ticking on one thread. The
scheduler is right and the executor header is stale. Fixing it is also claimed
by the P2 document; the two should not both silently take ownership.

Two more, briefly. Per-tick fault isolation is one `catch_unwind` in
`NodeRunner::run_tick` (`scheduling/primitives.rs:229`); `init()` and
`shutdown()` panics are caught separately (`scheduling/types.rs:596-625`), and
the compute worker wraps its dispatch in a second one
(`compute_executor.rs:199-206`). A profile that set `panic = "abort"` would
delete all of them — the workspace does not set it today
(`Cargo.toml:119-120`, which sets only `lto = "thin"`). And the main tick loop
runs on a phase-locked absolute grid rather than sleeping a period after the
work; the reason (a 100 Hz scheduler whose nodes took 3 ms ran at 77 Hz,
load-dependently, and nothing said so) is written at
`scheduling/scheduler/mod.rs:3914-3933`, with the overrun counter at `:241-243`
and the same figure restated on `TickState::next_deadline` at `:236-239`.

## Who actually depends on this crate

The "internal crate, use `horus` instead" framing is right as advice and wrong
as a description of the dependency graph. What is real:

- Seven crates in this workspace take `horus_core` as a normal path
  dependency: `horus`, `horus_manager`, `horus_net`, `horus_py`, `horus_cpp`,
  `horus_types` and `benchmarks`. `horus_macros` takes it as a dev-dependency
  only (`horus_macros/Cargo.toml:29`), and `horus_cpp` and `horus_manager` take
  a second, dev-scoped copy (`horus_cpp/Cargo.toml:35`,
  `horus_manager/Cargo.toml:127`). Counted as reference lines
  (`grep -rn 'horus_core::' --include=*.rs <crate>/src`, discounting
  `::horus::horus_core::`), the usage is `horus_manager` 171, `horus_net` 75,
  `horus_py` 55, `horus` 54, `horus_cpp` 54, `horus_types` 29, `benchmarks` 19,
  `horus_macros` 1. Only one of those, `horus`, is the facade; the other seven
  reach past it.
- Every project `horus new` generates lists `horus_core` as a direct path
  dependency. `write_horus_path_deps` iterates
  `["horus", "horus_core", "horus_library", "horus_macros"]` and emits a path
  dep for each that exists — `horus_manager/src/cargo_gen.rs:1066` for the
  single-package layout (called at `:116`), `:1808` for the workspace root's
  `[workspace.dependencies]`, and `:1979-1980` for each member's
  `horus_core.workspace = true`. So `use horus_core::…` compiles in a generated
  project, and nothing in the toolchain objects.
- It has to. `horus_macros` expands to `::horus_core::core::LogSummary`
  (`horus_macros/src/log_summary.rs:10`) — the only bare `::horus_core::` path
  it emits, and one that resolves only if the user's crate names `horus_core`
  directly. The rest of the generated code goes through
  `::horus::horus_core::…` (e.g. `horus_macros/src/node.rs:409`), which works
  because `horus` re-exports the crate at `horus/src/lib.rs:353-354`.
- The `test-utils` feature has a real consumer too: `horus_cpp` takes
  `horus_core` as a dev-dependency with `features = ["test-utils"]`
  (`horus_cpp/Cargo.toml:35`).

That is what the `#[doc(hidden)] pub use` block at `lib.rs:56-96` is for, and
the comment at `lib.rs:54-55` says so: those re-exports exist for
crate-internal use and for `horus_py` and macro-generated code cross-crate, not
for readers. **Do not delete a hidden re-export because nothing in this crate
uses it.** Grep the workspace first, not just this crate.

So: depend on `horus`, import from `horus::prelude::*`, and treat every
`horus_core` path as unstable. But write your commit message knowing that
breaking one is a breaking change for eight sibling crates and every generated
project, not an internal refactor.

## Doc comments that are attached to the wrong thing

Found while writing this file. They are wrong in the source, not only in the
rendering, so `--document-hidden-items` will not save you from them.

| Where | What it says | What it is attached to |
|---|---|---|
| `scheduling/scheduler/mod.rs:281` | "Central orchestrator: holds nodes, drives the tick loop." | `PresenceEntry` at `:284`. `Scheduler` at `:294` has no doc comment. |
| `scheduling/scheduler/mod.rs:3914-3933` | The tick-grid essay, explicitly about `compute_tick_sleep` | `effective_tick_period` at `:3939`, whose own doc starts at `:3934` and follows it with no blank line. `compute_tick_sleep` at `:3948` has none. |
| `communication/topic/dispatch.rs:90` | Invariant 1: "`Topic<T>` is `!Send + !Sync`" | Correct about `Sync`, wrong about `Send` since `mod.rs:3711`. The invariant it should state is single-thread *ownership*, which `Send` does not violate. |

Two more shapes of the same problem, worth knowing because they are not
individually fixable:

**Essays written as `//` on no item.** They render nowhere under any flag. Two
that are load-bearing: `scheduling/rt_executor.rs:366-429` (the cyclic-wait
rewrite, 64 lines, a banner comment above a group of constants — the next item
starts at `:431`) and `scheduling/scheduler/mod.rs:3097-3117` (the RT threading
model, attached to a `let` binding).

**No two places agree on how fast a topic is.** Fourteen files under `src/`
carry a `~N ns` figure in module prose
(`grep -rlE '//!.*~[0-9]+ ?ns' --include=*.rs horus_core/src`). For the topic
path specifically: `communication/mod.rs:9` says "~3ns (same-thread) to ~167ns
(cross-process)"; `communication/mod.rs:22-23`, `communication/pod.rs:54-55`
and `communication/macros.rs:26` say ~50 ns POD against ~167 ns serde, as do
the five `types/*_descriptor.rs` headers (`:4` in each) and
`types/action_chunk.rs:10`; `communication/topic/mod.rs:13-19` gives a
per-backend table running ~40 ns (FanoutShm) to ~85 ns (SpscShm);
`scheduling/event_executor.rs:33` says "~300 ns the transport delivers"; and
`dispatch.rs:21-24` reports ~105-118 ns cross-thread one-way for
`Topic<CmdVel>`. This README's own table, deleted in this rewrite, added three
more: 3 ns same-process fan-out, 10 ns same-process SPSC, 198-304 ns
cross-process (`README.md:83-87`, pre-rewrite). The "~3 ns same-thread" claim
sits particularly badly against `topic/mod.rs:7-9`, which
says every topic is SHM-backed and all backends are cross-process, and against
the table four lines below it whose fastest entry is ~40 ns. Only `dispatch.rs`
names the harness that produced its numbers — `topic_probe` and
`topic_probe --raw-ring` (`benchmarks/src/bin/topic_probe.rs`, the flag parsed
at `:618`), sharing threads, pinning, clock and ack protocol so the transport
is the only variable. Quote that one, with its conditions, or quote none.

## What this file does not fix

Writing it down does not publish it. Three things are still open, and this
README is the wrong tool for all three:

1. **Nothing deploys `target/doc`.** CI builds it (`ci.yml:345`) and throws it
   away. Until something hosts it, the command in
   [How to read the internals](#how-to-read-the-internals) is the only route,
   and it needs a nightly toolchain.
2. **`cargo doc` excludes `horus_manager`** (`ci.yml:345`). The reason is real:
   `horus_manager` declares `[[bin]] name = "horus"` at
   `horus_manager/Cargo.toml:26-27`, which would write `target/doc/horus/` on
   top of the `horus` library crate's output (`ci.yml:337-338`). Restricting
   the workspace doc build to library targets is the obvious candidate.
   **Unverified:** nobody has run `cargo doc --workspace --no-deps --lib` here
   to confirm it resolves the collision. Treat it as a lead, not a fix, and do
   not let a later edit promote it without someone running it.
3. **The hidden modules stay hidden.** That is deliberate — un-hiding them
   would present `horus_core` as a supported API, which it is not — but it
   means the rendered `horus` docs and this file are the only two places a
   reader can start. Keep them in step.

Every line number above is from the working tree as of this writing. The most
fragile are the four-digit ones in `scheduling/scheduler/mod.rs` (`:281`,
`:294`, `:3097-3119`, `:3914-3948`) and `communication/topic/mod.rs` (`:3711`,
`:3713-3719`, `:3721-3728`): one inserted function moves them all. A checker
that asserts every `pub mod` in `lib.rs` appears in the table above, and that
the `Send`/`Sync` sentence matches the impls, would catch drift in the module
list and in the contract — it would not catch drift in the line citations.

## Licence

Apache-2.0. See [LICENSE](../LICENSE).
