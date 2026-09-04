# Changelog

All notable changes to HORUS are recorded here.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## How versions are numbered

**The patch number increments once per release. The version string does not
encode the size of the change.**

0.4.0 -> 0.4.1 -> 0.4.2, regardless of whether a release carries a typo fix or
a shared-memory ABI break. This is deliberate and it is NOT
[Semantic Versioning](https://semver.org/spec/v2.0.0.html) — entries before
0.4.1 were numbered that way and reason about "minor rather than patch", which
no longer applies.

The consequence worth stating plainly: **you cannot tell from the version
whether an upgrade is safe in place.** Read the Compatibility section of the
release you are moving to. Where there is a shared-memory ABI change, it says
so at the top and in bold; `TOPIC_VERSION` also refuses a mismatched segment at
runtime and names both versions, so a mixed namespace fails loudly at startup
rather than corrupting liveness quietly.

## How this file was produced

Releases 0.1.5 through 0.2.2 shipped without a changelog. Their entries below
are **derived from the git history** — each line is a conventional-commit
subject from the range between two tags, grouped by its `feat:` / `fix:` /
`perf:` / `refactor:` / `docs:` prefix. They are therefore accurate to what was
committed, but they are not curated release notes: they use the wording the
author chose at the time, and commits that carried no conventional-commit
prefix are counted rather than listed. Where a release has such commits, the
count is stated so the omission is visible instead of implied.

From `Unreleased` onward, entries are written by hand.

## Release cadence

A release is cut whenever either is true:

- a user-visible fix has been on `main` for two weeks, or
- `git rev-list --count <last-tag>..HEAD` passes 50.

Whichever comes first. The point is that the distance between "fixed" and
"shipped" stays small enough to be uninteresting. It was 244 commits before
0.3.0 and 158 when that gap was first written down, which is how a user came to
install a binary missing a fix that had been in the tree for months, with
nothing to tell them so.

Every entry from `Unreleased` moves under the new version heading at cut time,
and `Unreleased` is left empty rather than deleted.

## Unreleased

## [0.4.1] — 2026-09-03

201 commits since 0.4.0 (2026-08-22), against a cadence rule that says cut at
50. Most of them arrived through two merge stacks rather than one PR at a
time: `main` requires six contexts with `strict: true`, so every merge
invalidates every other open PR and the queue drains at one PR per full CI
cycle.

Version numbers move 0.4.0 -> 0.4.1 across `horus`, `horus_core`,
`horus_manager`, `horus_types`, `horus_py`, `horus_macros` and `benchmarks`.
`horus_sys` moves 0.2.0 -> 0.2.1; it gained public API (`PiMutex`,
`PiMutexGuard`, `ResidencyPolicy`) and its layout is unchanged. `horus_net`,
`horus_cpp` and `horus_cpp_macros` stay at 0.1.0.

**Read the compatibility section before upgrading.** This is a patch number and
it is not a drop-in upgrade: the shared-memory ABI changed, and three
behaviours changed with it. Releases here are numbered by increment rather than
by the size of the change, so the version string is not the signal — this
section is.

### Breaking

- **core** — `Topic::<Tensor>::pool()` (`#[doc(hidden)]`, but `pub`) returns
  `HorusResult<Arc<TensorPool>>` where it returned `Arc<TensorPool>`. A pool
  file left behind by a build with a different `POOL_VERSION` or geometry can
  be neither opened nor recreated, and the infallible signature had only one
  way to say so: a panic, on the per-frame allocation path, out of constructors
  that already return `HorusResult`. There is no compatible spelling of the old
  signature that does not keep that panic. Callers add `?`; the neighbouring
  methods a caller is more likely to hold — `alloc_tensor`, `send_handle`,
  `recv_handle` — are unchanged, and no caller of `pool()` exists outside
  `horus_core` in this workspace. Pre-1.0, so it rides in the minor slot, as
  0.4.0's break did.

### Compatibility

- **Do not mix 0.4.0 and 0.4.1 processes on one machine's shared memory.**
  `TOPIC_VERSION` moves 3 -> 4. Every millisecond timestamp in the topic
  header — participant leases, `last_topology_change_ms`, `stall_since_ms` —
  moved from `CLOCK_REALTIME` to `CLOCK_MONOTONIC`. The fields did not change
  layout, size or alignment; they changed MEANING. A v3 process stamps a lease
  at ~1.75e12 (ms since 1970) and a v4 process stamps the same field at ~1e7
  (ms since boot), and nothing in the segment says which epoch a number came
  from. Mixed, they mis-judge liveness in both directions at once: the v3
  writer's leases look permanently expired to a v4 reader, whose slot is then
  reclaimed underneath a live process, and the v4 writer's leases look ~55
  years in the future to a v3 reader, which never reaps a crashed one. There is
  no in-band discriminator that could make this safe, so `Topic`'s open path
  refuses a mismatched segment outright and names both versions. Restart every
  node in a namespace together.

- **`send_blocking` now refuses on `FanoutShm`.** It already refused on
  `PodShm`. `FanoutShm` was classified as backpressured because
  `send_fanout_shm` has two `Err` returns, but neither is a full ring — one is
  endpoint-slot exhaustion and one is an oversized message — and the ring
  itself is drop-oldest (`send_pod` is documented "Never fails"). So
  `send_blocking` was returning `Ok` for a delivery nothing guaranteed, on the
  emergency-stop and motor-setpoint path its own documentation points at. The
  `NoBackpressure` error also used to recommend migrating TO `FanoutShm` to get
  backpressure, which moved the caller between two lossy backends and changed
  nothing.

- **`Topic.missed_count()` and `.dropped_count()` raise in Python.** They
  returned `0` when the topic lock was poisoned, so `0` meant both "no loss"
  and "could not read" — in the two methods whose entire purpose is telling
  those apart. Both return `PyResult<u64>` and raise `RuntimeError`. Callers
  that treated the return as infallible need a `try`.

### Added

- Loss counters on the Python and C++ bindings: `missed_count()` on the
  subscriber, `dropped_count()` on the publisher, both in `stats()`. A topic is
  lossy by design, and until now only Rust could see it.
- A priority-inheritance mutex (`PiMutex`) the 1 kHz thread can wait on without
  unbounded inversion.
- Action chunks, and a consumer that reports when one has expired rather than
  acting on it.
- `horus lock --check` for pin drift, and `--print-service` for the systemd
  unit `horus deploy` installs.
- `distribution.yml`: CI now executes `install.sh`, `install.ps1`,
  `uninstall.sh` and `uninstall.ps1` on Ubuntu, macOS, Windows, Git Bash and
  `debian:11`, and checks that the binary and the source come from one tag.
  None of these scripts had ever been run by CI on any platform.

- Camera frames carry a capture timestamp distinct from publish time.
  `timestamp_ns` is stamped when the descriptor is built, which for a camera is
  exposure + transfer + driver latency after the shutter — routinely 10-30 ms.
  Fusing a camera against an IMU on publish stamps aligns them on when frames
  *arrived*, not when they were *measured*.
- Python can read `Imu` orientation and its covariances. All six fields already
  crossed shared memory intact in the 304-byte POD, but `PyImu` exposed getters
  only for accel, gyro and `timestamp_ns` — `qz` did not exist, so the
  quaternion was write-only and the three covariance matrices unreadable.
- A failed C++ topic says why and counts its losses. `horus_publisher_*_new`
  answers failure with a null pointer, and the C++ `Publisher`/`Subscriber`
  turned that into a handle that looked constructed and discarded everything
  sent through it, with the reason horus_core had named thrown away.

### Fixed

- `install.sh` resolves ONE tag and uses it for both the binary and the source
  tree. They could previously come from different refs — a released binary
  against `main` source — with both reporting the same version.
- Windows: `horus build` emitted `C:\Users\...` into generated TOML, where
  `\U` is an invalid escape, so it failed for every Windows user; the CLI now
  runs on a 16 MB thread instead of the 1 MB main-thread stack it was
  overflowing; `install.ps1` is a native PowerShell installer.
- `uninstall.ps1` exited 1 after a completely successful uninstall — the last
  external command on the ordinary path is a `pip show` PROBE, which returns 1
  when the package is absent — and deleted a TOML-escaped path that never
  existed while leaving the real binary behind.
- RT grades were unreachable: `rt_report` measured jitter as the raw interval,
  so `RtGrade::Production` (p99 < 50us) and `Standard` (p99 < 500us) could not
  be met at any rate and `is_production_ready()` returned false on every
  machine, including a correctly configured PREEMPT_RT one.
- Replay at a speed other than 1.0 anchored the cadence grid on the unscaled
  period, so the first tick was the wrong length.
- Network topics were silently decimated to the export tick rate.
- An epoch flush could drag the `SpmcShm` cursor backward.
- `msggen` emitted `[float; 3]` verbatim (not a Rust type), `[String::new(); 2]`
  (which needs `Copy`), and accepted unknown array element types silently.
- Shared memory: abandoned regions are reclaimed inside a namespace still in
  use; a tensor pool whose creator died before writing its header is reclaimed;
  a lost `FanoutShm` attach no longer strands a subscriber; a region sibling
  `Topic` clones still point into is no longer unmapped.

- **core** — a tensor pool file this build disagrees with is now an error
  rather than a panic. `TensorPool::drop` never unlinks and the filename
  carries no version, so an in-place upgrade across a `POOL_VERSION` bump
  leaves a file the next run can neither open nor recreate; the registry
  `.expect()`ed on that, so `Topic::new`, `Image::new`, `PointCloud::new`,
  `DepthImage::new`, `CostMap::new`, `OccupancyGrid::new` and
  `TensorHandle::from_shape` panicked instead of returning the `HorusResult`
  they advertise. Inside a scheduler tick the panic was swallowed by
  `catch_unwind` — the node entered ticks, completed none, and the process
  still exited 0. The paths with no error channel to propagate into
  (`Topic::<Tensor>::recv_handle`, the spill read/write paths,
  `TopicMessage::try_from_wire`) answer `None` instead, and say why: one
  throttled `warn` per faulted topic per second, carrying the number of
  occurrences it suppressed, so a faulted topic is not delivered to an
  operator as an idle one.

- **cpp** — a malformed `.horus/config/params.yaml` no longer downgrades a C++
  node to the built-in safety limits. `horus::Params` was the only one of the
  three bindings that started anyway: Rust's `RuntimeParams::new()` returns
  `Err` and Python's `Params()` raises, but the C++ constructor took
  `unwrap_or_default()` and served `max_speed = 1.0` and
  `emergency_stop_distance = 0.3` under the operator's name, with no channel to
  tell the caller. `horus_params_new` now returns `NULL` on that path and the
  `horus::Params` constructor throws `horus::Error`.

- Over-aligned POD messages are no longer published through a misaligned typed
  store. `colo_eligible` picked the co-located geometry on `type_size` alone,
  but a colo payload starts 8 bytes into a cache-line-aligned slot and nothing
  rounded the slot up to `align_of::<T>()`. A 16-byte, 16-aligned message was
  therefore colo-eligible and stored at an address 8 mod 16 — UB, and a
  `movaps` fault in release for the shapes LLVM vectorises.
- The fanout owner refuses to build a POD slot smaller than the message.
  `compute_slot_size` clamped with `.min(4096)` instead of refusing, `init_owner`
  wrote that geometry into the meta block unchecked, and `try_send_pod` memcpys
  `size_of::<T>()` bytes with no length check anywhere on the path — so every
  send of a POD larger than the cap ran off the end of its slot.
- The colo fast path fences its seqlock write phase. The WRITING marker was
  stamped with `store(Release)` and the payload written with nothing in
  between; a release store orders only what precedes it, so a reader could
  observe the payload first and return a mixture of two messages.
- The seqlock fast-forward is committed before the read attempt.
  `seqlock_consume` billed the lap gap to `skipped` as soon as it
  fast-forwarded but stored `tail` only on success, so an exhausted attempt
  loop left the skip counted and the cursor stale — and the next call
  re-derived and re-billed the same gap.
- A participant slot is only freed once its thread has ended. `register_role`
  took any lease-expired slot belonging to this process and decremented the
  role counter under it, but an expired lease is not evidence the owner is
  gone: housekeeping refreshes every 64 polls, so a subscriber polling at 1 Hz
  sits expired for roughly 92% of every cycle.
- The `/dev/shm` reaper runs. `cleanup_stale_namespaces()` opened its scan with
  `if !dir_name.starts_with("horus_sid") { continue; }`, and nothing has
  produced a `horus_sid*` directory since the flat-namespace model — so every
  real namespace was skipped and the reaper freed nothing.
- `Node::on_error()` is guarded on all four executor paths. It was called bare
  in the RT, compute, async and event executors, each time on the line before
  `apply_failure_policy_after_panic()`; a panic inside a user's `on_error`
  unwound out of the executor instead of reaching the failure policy. The
  main-thread scheduler already ran the identical call inside a guard.
- Telemetry export failures are reported. Both production callers were
  `let _ = tm.export();` and nothing inside `export()` logs, so an unwritable
  `file://` path, an unbound UDP socket or a dead HTTP export thread was
  silently discarded for the life of the run.
- `RtConfig::apply()` no longer reports SCHED_FIFO applied when no priority was
  given. The scheduler-policy test was nested inside the priority test, so a
  config that set only `.scheduler(RtScheduler::Fifo)` issued no
  `sched_setscheduler` at all and still answered `Ok(FullSuccess)`.
- `horus_net` keys per-peer metrics on the peer. `record_send` and `record_recv`
  both passed this node's own id, so `snapshot().peers` could only ever
  describe us and no genuine peer ever got a row; refused sends are no longer
  counted as sent.
- `horus topic hz` and `horus topic bw` say when a topic stalls instead of
  freezing the last figure. Every print was gated on "the counter moved", so a
  publisher that died mid-measurement left `Rate: 20.19 Hz` on screen with the
  cursor parked on it — indistinguishable from a live readout.
- `world_to_grid` and `grid_to_world` honour `origin_theta`. It is stored on
  both `OccupancyGridDescriptor` and `CostMapDescriptor`, documented as "map
  origin orientation (radians)" and exposed by a getter on both wrappers, and
  neither conversion ever read it — both did translation only, so every
  rotated map silently mapped to the wrong cell.

### Documentation

- `Topic::send_blocking`'s real error contract is on the method users read.
  The corrected prose about broadcast backends had landed only on
  `RingTopic::send_blocking`, which is `pub(crate)` and renders nowhere; the
  public method still promised delivery it does not make on those backends.
- `Scheduler::tick_once` and `run` list the errors they really return. The
  `# Errors` sections named three `NodeError` variants none of which can come
  out of either — init errors are swallowed before a caller sees them, and
  `Node::tick` cannot fail.
- The README asks for issues and PRs rather than running a hardware survey.
  The old section asked readers to write in with their platform, control rate
  and what broke, routed to Issues, Discord or email.

### Testing

- The Python suite runs in CI for the first time: 413 test functions, 442 cases.
  Nothing had ever invoked pytest, and `--exclude horus_py` appears in every
  other Rust gate, so the binding layer had no coverage at all.
- 55 `horus_core` tests that were compiled but never executed now run — the
  cross-process chaos suites, `ipc_torture`, and the kill-9 reconnect test.
- A deterministic reproducer for the topic-attach SIGBUS (#144): a publisher on
  a `CmdVel` topic sending in a loop while a second thread repeatedly attempts
  `Topic::<Imu>::new` on the same name. ~40 lines and 6 failures out of 6,
  against the original's 2 in 5 across two schedulers and 500 ticks.
- The SHM magic-corruption test asserts the rejection it only named. It
  corrupted a region, attached, and printed one of three outcomes with a
  comment saying all three were acceptable; it now pins the refusal to the
  joiner's `HeaderInitTimeout` rather than to a bare `is_err()`.
- Three cross-thread topic tests now run. `topic_cross_thread_1p_multi_c_spmc`,
  `..._multi_p_multi_c_mpmc` and `..._mpmc_pre_initialized_99_percent` were
  `--skip`'d by six of the eight CI invocations that build the target, and the
  other two filter by name for something else.

## [0.4.0] — 2026-08-22

The first release since 0.2.2 (2026-07-19). **0.3.0 was tagged and never
published** — the tag was cut locally, no GitHub release was created, and
`install.sh` reads `releases/latest`, so every user installing HORUS in that
window got 0.2.2 with nothing to tell them so. That gap is the thing this
release exists to close, and the tree had drifted 69 commits past the unshipped
tag by the time it was cut.

Version numbers move 0.3.0 -> 0.4.0 across `horus`, `horus_core`,
`horus_manager`, `horus_types`, `horus_py`, `horus_macros` and `benchmarks`.
`horus_sys` moves 0.1.0 -> 0.2.0 because its shared-memory header changed (see
Compatibility). `horus_net`, `horus_cpp` and `horus_cpp_macros` stay at 0.1.0.

Minor rather than patch: there is one breaking API change and three behaviour
changes. Pre-1.0, so they ride in the minor slot.

### Compatibility

- **Do not mix 0.2.x and 0.4.0 processes on one machine's shared memory.**
  `TopicHeader` gained two producer-side words, carved out of previously
  reserved padding. The struct is still exactly 640 bytes and every existing
  field is at its old offset, so the change is invisible to a reader that does
  not look at them — but a 0.2.x publisher and a 0.4.0 subscriber disagree
  about whether those bytes mean anything. Restart every node in a namespace
  together.
- **Two built-in message layout hashes changed value**: `Clock`
  (`0xc25212af` -> `0x044c9261`) and `SimSync` (`0x3959875d` -> `0x6d289920`).
  These are corrections — the old numbers described structs without their
  padding fields, i.e. structs that do not exist — but they are the numbers
  `horus msg hash` prints, so anything that recorded them needs updating. The
  other 22 built-ins are unchanged.

### Breaking

- **`horus_types`** — `RateRequest::topic()` is renamed `topic_name()`. The old
  name collided with the new `Type::topic(name)` opener that every message type
  now carries. No caller outside `horus_types`' own tests existed.


### Added

- **manager** — `horus fmt` and `horus lint` state that they cover C++. Both
  have run `clang-format` and `clang-tidy` for some time; the help said
  "Rust + Python" and "clippy + ruff/pylint", so the C++ half was reachable
  only by trying it.

### Changed

- **manager** — `horus run --sim` is now a hard error when the simulator it
  needs cannot be launched. It previously logged one warning and then ran the
  node against real hardware. On a bench with actuators attached, that is the
  difference between a simulation and a motion command, and the flag was
  explicitly asked for.
- **manager** — the `--sim` help names `[hardware]` entries with `sim = true`.
  It named `[sim-drivers]`, which the manifest has carried as legacy since the
  flag moved.

### Fixed

- **core** — a crashed owner's shared-memory region hung every process that
  opened it afterwards, so one crash took out introspection for the whole
  namespace until `/dev/shm` was cleared by hand.
- **core** — a lapped subscriber lost messages while every counter read zero,
  which made the loss invisible to exactly the tooling that exists to find it.
  A lapped *broadcast* subscriber received nothing at all.
- **core** — a consumer's read position could be pulled backward by a
  migration, replaying messages it had already handled.
- **core** — a node whose registry was wiped reported `Healthy` at 0 ticks.
- **core** — a topic created before the network hook was installed stayed
  invisible to it, so whether a topic replicated depended on construction order.
- **core** — `send_blocking` ignored its deadline until it had already blocked,
  and the lossy send blocked for 100 ms rather than dropping — the one thing a
  lossy send exists not to do.
- **core** — a generic type name was stored mangled and then failed to match
  itself, so two ends of the same topic disagreed about its type.
- **core** — reading local presence deleted the fleet's remote host records.
- **core, sys, cpp** — a print on a broken stream killed the thread doing the
  printing. A robot's stdout breaks routinely: the supervisor exits, or an
  operator pipes `horus run` into `head`. On a background thread that left the
  process running with a subsystem silently dead.
- **rt** — a closed stdout no longer kills the real-time thread, and the
  per-tick diagnostics that were destroying the log are throttled.
- **scheduler** — a node name can no longer forge lines in the runtime's output.
- **shm** — a publisher restart no longer deafens its subscribers.
- **services, actions** — a panic in user code no longer kills the server.
- **net** — an e-stop that reached nobody was indistinguishable from one that
  worked.
- **net** — the default import mode admitted nothing, so no remote data ever
  arrived; the import guard could not tell what the robot publishes; outbound
  replication stopped once the topology settled; and a peer was declared
  link-lost before it had time to answer.
- **net** — the presence receiver read the namespace differently from everyone
  else, and three further sites asked the environment directly instead of going
  through the accessor.
- **manager** — `horus clean --shm --force` wiped every robot on the machine,
  not the current project's namespace.
- **manager** — `horus check` failed on this repository over one of its own test
  fixtures, reported a package's own modules as missing, and said a module was
  missing without naming it.
- **manager** — `horus topic echo` printed hex for 83 of 92 message types;
  `topic echo` and `topic pub` could not be used together; `topic pub`
  published a payload nothing could ever read; `topic list` called every topic
  active, including empty ones; `topic info` said "(none)" for a topic with live
  endpoints; and `horus service list` reported 0 servers for a service that was
  serving.
- **manager** — the CLI printed instructions naming commands it does not have.
- **manager** — `horus doctor` called a ninja-only toolchain ready and the build
  then demanded `make`: `cmake` was invoked with no `-G`, so it always took the
  platform default generator. The generator is now selected, which makes
  doctor's claim true rather than narrowing it (DOC-1).
- **manager** — `horus topic echo` still dropped messages behind a fix that
  claimed to have ended it. Every slot in a batch reported the header's live
  head rather than the ordinal actually read, so a caller resuming from the
  first slot skipped the rest of the batch (LIVE-5).
- **python** — every `astype()` to a different dtype panicked with
  `PyBorrowMutError`, taking `to_float32()`, `to_float16()`, `to_int32()` and
  `to_uint8()` with it. Same-dtype conversions returned early and worked, which
  is why the API looked healthy.
- **python** — a failing Python node reported its exception without the
  traceback, and the shipped Python example panicked on every tick.
- **discovery** — `horus topic list` deleted a restarted node's topic.
- **packaging** — the root `Dockerfile` is now actually built in CI (the
  existing workflow built `tests/docker/Dockerfile.<distro>` instead, so it was
  never covered), and the devcontainer is repaired. Shell-completion install is
  verified against throwaway `HOME`s for bash, zsh and fish; the zsh `fpath`
  case that shipped a file nothing sourced is fixed, and `uninstall` now removes
  what `install` wrote (TOOL-1, TOOL-2, TRUST-2).
- **manager** — the hand-written help template no longer drifts from clap, the
  four mis-stated command rows resolve, and manifest errors name the file and
  location on the single-file path as well as the workspace path
  (HELP-1, HELP-2, HELP-5, CFG-3, FLAG-2).
- **core, manager** — the message registry no longer stops at the built-in
  types, and topic owner attribution reaches the zero-copy specialisations
  (`Image`, `PointCloud`, `DepthImage`, `Tensor`), which still read the
  constructor-captured owner (LIVE-4, LIVE-6).
- **scheduler** — the timing report printed each node's total tick count under
  its "Misses" header, so a node that shut down after 249 ticks having missed
  nothing reported 249 misses. Ticks and deadline misses are now separate
  columns (LIVE-7).
- **manager** — generated projects name their node after the project in every
  template, not `controller` (LIVE-8).

### Testing

- Contract suites added for the manifest, message-generation interop, the
  install URL, completion install, translations and container images, so these
  are gated rather than assumed.
- `cargo test -p horus_net` failed on every run for four reasons; every
  `cargo test` ran in the same shared memory a robot uses; two presence stress
  tests corrupted each other's directory; the dual-write routing test read a
  process-global counter; and the live introspection poll gave up before a
  loaded machine could answer. All fixed — these were failures of the tests,
  not of the product, and each one taught people to ignore a red suite.

## [0.3.0] — 2026-08-21 (tagged, never published)

This version was bumped and tagged locally and no GitHub release was ever
created for it, so nobody installed it. Its entries are kept because the code
changes are real and shipped in 0.4.0; the version number is the part that
never reached anyone.

Cut as the first release since 0.2.2 (2026-07-19), which the tree was 244
commits ahead of. Users installed a binary that stale and had no way to tell — the gap
this release exists to close, along with the cadence note below.

Version numbers move 0.2.2 -> 0.3.0 across `horus`, `horus_core`,
`horus_manager`, `horus_types`, `horus_py`, `horus_macros` and `benchmarks`.
`horus_sys`, `horus_net` and `horus_cpp` stay at 0.1.0; they are versioned
independently and nothing in this release changes their public surface.

Minor rather than patch: there are new features and several behaviour changes
(listed below). Pre-1.0, so behaviour changes ride in the minor slot.


### Added

- **manager** — `post_run` and `post_build` lifecycle hooks. Every phase now has
  both a `pre_` and a `post_` hook; previously only `test` had teardown, so
  there was no way to release a bus or park an arm after a run. Both fire on
  the success and failure paths.
- **manager** — `horus check --json` now emits a `diagnostics` array with
  message, file, line and severity, instead of only a boolean and a count.
- **manager** — `horus schema` prints the JSON Schema for `horus.toml`.
- **manager** — unknown keys in `horus.toml` are reported with a suggestion
  rather than silently ignored.
- **core** — `Topic::new_checked(name, hash)`, plus `LAYOUT_HASH` and a
  `Type::topic(name)` helper on every `message!` type. The topic open path
  checked only the message's short type name and its size, so two builds of
  `Pose { x, y }` and `Pose { y, x }` shared a topic and swapped the coordinates
  with no error. The checked constructor rejects it; `Topic::new` is unchanged
  and unchecked peers are never rejected.
- **horus** — `horus::types` now re-exports both `horus_types` and
  `horus_core::types`, and `horus::error` / `horus::HorusResult` /
  `horus::topics` are reachable directly. `use horus::types::Tensor;` failed
  with "no `Tensor` in the root", and twelve doc sites told readers to write
  `horus::horus_core::…` — naming an internal crate the facade exists to hide.
- **manager** — `horus man` renders a man page from the same clap tree the CLI
  is built from; `install.sh` now places it. HORUS shipped none at all, on a
  tool whose installer already placed a completion script.
- **packaging** — a root `Dockerfile` (two-stage, pinned to the workspace MSRV,
  shipping the man page and completions) and a `.devcontainer` that builds its
  builder stage rather than duplicating the dependency list. Both set
  `--shm-size`, since the default 64 MB `/dev/shm` is too small for image or
  point-cloud topics.
- **python** — PEP 561 `py.typed` marker, so the shipped `_horus.pyi` stubs are
  actually used. Type checkers previously skipped the package entirely.

### Changed

- **python** — `Topic.name` now reports the endpoint the topic was actually
  created under. It previously returned the name derived from the message type,
  so `Topic(Imu, endpoint="imu.data")` reported `imu` while its data lived at
  `/dev/shm/<ns>/topics/imu.data`. The property is also what `log_ipc_event`
  records, so verbose IPC logs named a topic that did not exist. `Topic(Imu)`
  with no endpoint is unaffected.
- **rust** — `horus::prelude` re-exports `TransformFrame`, `Transform`,
  `FrameBuilder` and `TransformFrameConfig`. `horus_py` and `horus_cpp` both
  depend on `horus-tf`, so Python and C++ reached the coordinate-frame tree out
  of the box while Rust — which had only `TransformStamped`, the message —
  required hand-adding a pinned git dependency.
- **manager** — `horus new` names a project's node after the project
  (`my_robot_controller`) in all three languages. Every generated project was
  previously named `controller`, so two HORUS projects on one machine collided
  by default and triggered the runtime's own duplicate-node-name warning. The
  `node!` template had no name at all and was named after its struct.

### Fixed

- **discovery** — a node that was SIGKILLed and restarted in the same namespace
  became invisible to introspection: `horus topic list` reported "No active
  topics found" and `horus topic echo` hung, while `horus node list` showed the
  same node Running and Healthy and its log said it was publishing. The `.meta`
  sidecar records the creating process's PID and is not rewritten when a later
  process attaches, so the restarted node inherited its own dead PID — and
  `horus_manager` acts on a dead PID by unlinking the backing file, so the
  command meant to observe the system deleted a live node's topic. Liveness now
  comes from the `flock` every holder of the region already takes, which the
  kernel releases even under SIGKILL.
- **python** — `Miss.SAFE_MODE` never called a Python node's
  `enter_safe_state()`. The adapter implemented only `init`/`tick`/`shutdown`,
  so the hook fell through to the no-op default while the scheduler correctly
  reported the deadline misses and degradations that were supposed to trigger
  it. Rust and C++ nodes were unaffected.
- **python** — `Tensor.astype()` panicked with `PyBorrowMutError` for any
  conversion that actually changed dtype, taking `to_float32()`, `to_float16()`,
  `to_int32()` and `to_uint8()` with it. A same-dtype conversion returned early
  and so appeared to work.

- **core** — `Miss::SafeMode` called `enter_safe_state()` on every deadline miss
  rather than on entry into safe mode. A node sleeping 5 ms against a 1 ms
  deadline was safed 18 times across 17 ticks, alternating between commanding
  and safing. It now fires once per degradation episode, and the latch clears
  when the node meets its deadline again.
- **core** — errors printed their message twice when a context wrapper and its
  source rendered the same text.
- **core** — `message!` recursed until the compiler gave up on a malformed
  definition; it now emits a diagnostic naming the supported forms.
- **core** — RT degradation messages were suppressed unless verbose mode was on.
- **manager** — cargo's warnings were discarded on a build that succeeded, so
  `#[must_use]` on `NodeBuilder` never reached the user through `horus build`.
- **manager** — `horus topic echo` went deaf after the ring buffer wrapped.
- **manager** — `--net` set an environment variable read in one place and did
  not enable the `net` feature, leaving `horus_net` unreachable.
- **manager** — `horus deploy` excluded its own binary and could build the wrong
  manifest.
- **manager** — `setup-rt` tried to install a package that does not exist on
  Ubuntu, and ran `sudo apt` without asking.
- **core** — type mismatch and type-size mismatch on a topic were reported as
  "Communication serialization failed", behind two stacked prefixes. They now
  name the topic and say what actually differs.
- **manager** — `horus msg hash` used `DefaultHasher`, whose output std does not
  guarantee between Rust releases, so the hash changed on a toolchain upgrade
  and reported every message as modified. It also hashed a different canonical
  form from the runtime, so it printed a number unrelated to the one in a
  layout-mismatch error. Both now use FNV-1a over `Name|field:Type|…`.
- **examples** — nine of the ten shipped examples did not compile, including the
  one the README names as the starting point. Causes: `hlog!`/`hlog_every!`
  called without a level (neither macro had a failure arm, so the error blamed
  the format string), `use horus::DurationExt` when the trait was only in the
  prelude, `[drivers] imu = …` generating a `horus` feature that has never
  existed, and API drift (`handle.cancel` → `canceled`, `SyncActionClient` →
  `ActionClient`, `TransformFrame` used without declaring `horus-tf`, a message
  read with `read_latest()` without `#[fixed]`). All ten build now.
- **examples** — every manifest carried `author` (the field is `authors`, so
  attribution was dropped) and `language` (inferred; no effect), plus a
  `horus_library` dependency that only one example referenced and that no
  registry serves.
- **horus_cpp** — none of the six C++ examples built. Three copy-initialized a
  `Scheduler` from its builder chain, which `scheduler.hpp` documents as not
  compiling; `pub_sub_demo` was missing from the CMake list entirely, so nothing
  ever compiled it; `camera_publisher` used a `CameraInfo` shape that no longer
  exists. The CMakeLists used `find_package(horus QUIET)` and linked only
  `if(horus_FOUND)` — HORUS ships no CMake package config, so nothing linked and
  every example failed with 82 undefined references instead of a message. All
  six build, link and run now.
- **docs** — `examples/README.md` said "both Rust and Python" and never
  mentioned the six C++ examples, so a C++ reader concluded there were none.
- **benchmarks** — `dds_comparison_benchmark` fabricated ROS2/CycloneDDS/FastDDS
  /iceoryx results when built without the `dds` feature (the default): a full
  percentile distribution, confidence bounds included, computed from two
  hardcoded constants and written into the JSON report beside real
  measurements with nothing to distinguish them. `BenchmarkResult` now carries
  a `provenance` field — `measured` or `literature` with a citable source.
- **docs** — the README led with "575x faster than ROS2" and a comparison table
  using ~50/100/70 µs for ROS 2. Those figures appear nowhere in the
  repository; its only ROS 2 reference is REP 2014's ~5 µs, ten times lower.
  Two of the three HORUS rows were producer-side `send()` latencies compared
  against a competitor's end-to-end number. The table now states what each row
  measures and names the benchmark that reproduces it, and the ROS 2
  comparison is stated as ~30x against the repository's own cited source. The
  concepts doc called it "550-875x … in measured pub/sub benchmarks", which
  asserted a measurement that does not exist.
- **manager** — the five native-tool proxies (`cargo`, `pip`, `cmake`, `conan`,
  `vcpkg`) answered `--version` and `--help` themselves. Inside a HORUS project,
  where `horus env --init`'s shell functions delegate, `cargo --version` printed
  `horus-cargo 0.2.2` instead of `cargo 1.97.1`.
- **manager** — the cmake proxy appended `-B <build-dir>` to every invocation,
  including modes that take no build directory: `cmake -E echo hi` printed
  `hi -B /…/cpp-build`. `-E`, `-P`, `--build`, `--install` and friends now pass
  through untouched.
- **install** — `horus env --init` edits `.bashrc`, `.zshrc` and fish's
  `conf.d`, and the installer ran it with output discarded, so the user was
  never told their shell files had been changed. It now says what it did and
  honours `HORUS_NO_SHELL_INTEGRATION=1`.
- **docs** — the `horus env` page named three shadowed tools; there are six. It
  now lists them, names every file the command writes, and documents the opt-out
  and the uninstall.
- **docs** — the install command in all six READMEs pointed at a branch that
  does not exist and returned HTTP 404.
- **docs** — the Configuration Reference documented 4 of the 15 tables
  `horus.toml` accepts; the remaining 11, including `[hardware]` and
  `[network]`, are now covered.

### Known issues

- **tests** — `cross_language_ipc` is reliable on its own (22/22) but fails
  occasionally in a full workspace run. Its loops race a wall clock while ~20
  test binaries and their Python children compete for the machine. Documented in
  the file, with why the obvious fixes do not work; the real fix is to have each
  side signal the other instead of racing a clock.
- **core** — `recv()` is documented FIFO but can return an older message after a
  newer one once the ring laps a slow subscriber. Observed backward jumps are
  exactly `capacity - 1`. Reproduce with
  `cargo test -p horus_core --lib recv_can_return_an_older_message -- --ignored`.

## [0.2.2] — 2026-07-19

### Added

- **tensor** — add TensorHandle::from_shape ergonomic constructor
- **horus_cpp** — action/service FFI server drivers + on_shutdown lifecycle
- **fanout** — align both backends on lock-free drop-oldest (latest-wins) seqlock
- ready-dispatch executor for automatic BestEffort parallelism

### Fixed

- **py** — build_messages() fails clearly for pip-installed users
- **record** — capture RT-node inputs (record export was empty for RT nodes)
- **py** — Tensor topic round-trip + DetectionList.append accept horus.Detection
- **cargo_gen** — inject serde so message! projects compile
- **record** — include per-tick snapshots in JSON export
- **run** — stream live output from multi-node Python launches
- **rt** — RT nodes ran at half their configured rate
- **cli** — repair `horus msg` and `horus deps` on a normal install
- **cpp** — make C++ projects build, link and honor their tick rate
- **cpp** — make the C++ pipeline reachable from detection and build
- **install** — repair the install -> first-run path end to end
- **services** — don't panic server thread on malformed client response_topic
- **topic** — drop tautological SpscShm||SpscShm assert + stale intra names
- **replay** — derive synthetic replay tick period from recorded wall-clock span
- **py** — delete_recording honors documented '-> bool' contract (return found/not-found)
- **py** — on_error must not swallow tick failure — re-raise so FailurePolicy escalates
- **net-estop** — wire the SEND path with a rising-edge anti-storm gate
- **security** — gate plugin execution on out-of-repo trust (SEC2 + SEC3)
- **manager** — preserve comments/formatting on horus add & remove (toml_edit)
- **manager** — preserve driver crate version (F1) and skip non-Cargo member deps (F4)
- **manager** — keep git source on workspace-member deps that also have features
- **manager** — route dual-listed packages by project language (F6)
- **manager** — write horus.lock and horus.toml atomically (temp + rename)
- **manager** — fail on signal-killed subprocesses instead of reporting exit 0 (F9)
- **registry** — verify Ed25519 signature on the pre-built-binary install path (SEC1)
- **actions** — make async ActionClient await_result work standalone + count node metrics (F-ACT1/F-ACT2)
- **scheduling** — wire the external emergency-stop hook to the scheduler (SCHED-H1)
- **topic** — reuse intra-fanout endpoint slots so re-registration can't panic (COMM-H1, intra half)
- **topic** — reclaim spilled TensorPool slots so FanoutShm stops dropping large messages (COMM-H3/F-MEM2)
- **python** — keep tensor slot alive under zero-copy numpy/torch views (F-PY1)
- **memory** — release grid tensor when cost alloc fails in CostMap::new_on (F-MEM3)
- **scheduling** — feed non-RT watchdog nodes to stop spurious e-stop (SCHED-H2)
- **services** — PID-scope ServiceClient ids to stop cross-process response theft (F-SVC1)
- **memory** — reuse pool region by physical capacity, not last logical size (F-MEM1)
- **horus_cpp** — route action results/feedback per client-PID (F2)
- **horus_cpp** — close service handler heap overflow (F1) + wire-cap alignment (F4)
- **ipc** — multi-subscriber zero-copy UAF (COMM-H2) + try_retain generation-ABA
- **horus_core,sys,py** — stub-audit fixes — failure_policy, actions, honest errors
- **rt** — make .no_alloc() usable — exempt a node's first tick (warmup)
- **rt** — stop per-tick alloc+leak in .no_alloc() instrumentation; prove hot path
- **fanout** — deliver oversized serde messages on FanoutShm via pool spill
- **memory** — validate SHM header geometry in TensorPool::open (M-E)
- **scheduler** — operator pause on a main-thread node must persist (Sched2)
- **safety** — make print_line non-fatal on broken stdout
- **memory** — reject integer overflow in TensorPool offset+size and align_up
- **memory** — bounds-check slot_id and add write-path generation check in TensorPool
- **safety** — isolate fault-path node callbacks so one node's recovery panic can't kill all nodes
- **safety** — latch e-stop before logging so a broken stderr can't skip it
- **topic** — non-POD multi-subscriber broadcast lost all but one subscriber
- **fanout** — drop ring-owned non-POD payloads at FanoutIntra teardown
- **topic** — early cross-process subscriber no longer loses buffered messages (softmata-brain bug #2)
- **topic** — SHM multi-subscriber is BROADCAST, not competing — fixes the detector, resolves the "concurrent consumer-join" (softmata-brain 1327)
- **topic** — CAS slot-claim in MP send paths — eliminates overshoot data corruption
- **topic** — SpmcShm consumer-join double-delivery via flush-on-resync (softmata-brain 1327, consumer side)
- **topic** — unify SpscShm onto the MpscShm atomic-claim protocol — eliminates multi-producer convergence loss (softmata-brain 1327)
- **topic** — preserve consumed frontier across same-data-plane resync — COMPLETES 1327 sp->mp fix, lifts DO NOT MERGE
- **scheduler** — rebuild dependency graph after RT/executor node partition (hang)
- **tensor_pool** — eliminate concurrent double-allocation (brain 1333)
- **topic** — role=Both query methods read local counters, not stale SHM header
- **topic** — reliable process_epoch propagation on migration (multi-producer delivery)
- **topic** — recv drained-detection overshoot — the real cross-thread hang
- **topic** — FanoutShm backend_matches_mode bug + broadcast-semantics tests
- **topic** — drain_in_flight 100us budget spuriously aborts migrations under load
- **rand** — complete rand 0.8->0.9 migration left incomplete by dependabot
- **security** — patch RustSec vulns + migrate pyo3 0.27->0.29 (closes Dependabot #25-#28)
- **security,ci** — patch CVEs, add audit gate, drop dead horus_library CI steps
- **clippy** — resolve CI-gating lint errors (-D warnings)
- **memory** — reclaim mmap pool data region via per-slot reuse
- auto-detect WSL2 and relax network timeouts
- resolve CI test stage failures
- disable incremental compilation in CI test job to reduce disk usage
- resolve clippy errors in default-features compilation path
- suppress unused macro warning, remove -D warnings from doc job
- remove assert!(true) and fix RUSTDOCFLAGS variable expansion in CI
- suppress unused assignment warning in chaos_cross_process test
- resolve remaining clippy warnings for CI
- rustfmt alignment in rigorous_e2e
- replace loop-match with while-let in rigorous_e2e test
- resolve all clippy -D warnings for CI

### Changed

- **horus_py** — rename dlpack_utils.rs -> tensor_convert.rs
- **topic** — adopt eager header.tail flush (now safe post-CAS); load-robust fanout test
- remove horus_library — decomposed into horus_types + horus-tf + horus-robotics

### Documentation

- **install.sh** — fix stale 'release' branch references in header comment
- correct #9 diagnosis — real intermittent race in shipped wheel
- **topic** — retire stale intra/DirectChannel comment terminology
- **topic** — note the 64B colo slot geometry is the cross-language wire contract
- **core** — correct is_safe_state / severity comments to match runtime
- **py** — fix Node.order docstring default 100 -> 0 (matches code)
- **test-harness** — document serial integration runs; drop the .cargo/config approach
- **horus_core** — fix HorusResult doctest (duplicate fn definition)
- **horus** — fix broken Quick Start doctest (undefined CmdVel)
- **tensor_pool** — correct pop_and_claim crash-recovery caveat
- **install.sh** — explain why pre-built binary may 404

_99 further commit(s) in this range carried no conventional-commit prefix and are not summarized here._
## [0.2.1] — 2026-03-30

### Added

- launch system + PREEMPT_RT integration + observability wiring

### Fixed

- remove broken RUSTFLAGS variable expansion in clippy CI job
- align comment formatting for CI rustfmt parity
- resolve all CI failures — check, fmt, clippy clean
- update installer and docs to use GitLab URLs, remove stale API references
- simplify CLI integration test scripts for GitLab YAML compliance
- quote YAML strings with :: to prevent mapping parse error in CI

### Changed

- switch horus-robotics and horus-tf from path deps to git deps (GitLab)

_1 further commit(s) in this range carried no conventional-commit prefix and are not summarized here._
## [0.2.0] — 2026-03-23

### Added

- production fan-out readiness, FanoutRing/ShmFanout backends, polyglot drivers, sim-real integration
- comprehensive test coverage across core, library, manager, and Python bindings
- add progress bar and spinner to install.sh
- BudgetPolicy enforcement, safety battle tests, dead code cleanup
- one-line installer for all platforms
- add LogSummary to detection, tracking, and landmark message types
- add Docker-based multi-distro user workflow tests
- cross-platform uninstall, acceptance tests, harden Windows CI
- **examples** — add 5 new examples covering key ROS2 migration workflows
- battle-test horus for production — 213 new tests, 7 bug fixes, service redesign
- **messages** — add AudioFrame type for microphone/speech/anomaly detection
- **api** — add ergonomic constructors for PointCloud and DepthImage, hide TensorDtype
- **observability** — unified SHM-based backend for metrics, discovery, and control
- **horus_manager** — add machine-readable doc extraction, C++ toolchain wiring, and DX improvements
- **horus_core** — wire per-node priority, core, watchdog into executors
- **horus_core** — RT executor thread pool for independent chains
- **horus_core** — wire ReplayClock into replay_from + record/replay tests
- deterministic parallel tests + Python time API bindings
- **horus_core** — determinism proof — bit-identical outputs across runs
- **horus_core** — add .priority(), .core(), .watchdog() to NodeBuilder
- **horus_core** — topic injection during replay + execution order tracking
- **horus_core** — ReplayClock for deterministic record/replay
- **horus_core** — wire dependency graph into deterministic tick loop
- **horus_core** — dependency graph for deterministic parallel ordering
- **horus_core** — clock abstraction + ambient time API (determinism blueprint phases 1-3)

### Fixed

- install.sh fallback to no-LTO when LLVM crashes (SIGILL)
- rewrite install.sh — remove fake progress bars, use cargo's native output
- repair horus_manager tests (2901 pass), auto-format all files
- repair 21 horus_manager test compilation errors (2901 tests now pass)
- Docker tests build horus_manager inside container (fixes glibc mismatch), fix horus new flag
- add missing response_topic field in horus_manager service call (caught by Docker test)
- dead code cleanup, wire format_bytes, add comprehensive tests
- parallel-safe test infrastructure, CI fixes, and codebase hardening
- **horus_py** — add missing AudioFrame arms in backend_type and metrics dispatch

### Documentation

- **horus_core** — clarify tick_once() sequential execution within steps

_162 further commit(s) in this range carried no conventional-commit prefix and are not summarized here._
## [0.1.9] — 2026-02-16

### Added

- Improve DirectChannel with thread-local indexed design (Issue #45)

### Fixed

- add bounds check to tensor pool data_slice before from_raw_parts
- preserve serialization error context in QueryError
- correct tick_with_timeout doc to not claim panic handling
- correct macOS error message to not reference /dev/shm
- replace Cell<u32> with AtomicU32 in ShmTopic for soundness
- add Drop impl for RealIoUringBackend to close socket fd
- **ci** — add cargo clean before test to avoid linker errors
- **ci** — exclude horus_manager from doc build to avoid filename collision
- feature-gate zenoh-only code to pass clippy
- resolve CI failures after horus-ros2 extraction
- **ci** — Use single-threaded tests to avoid discovery topic race
- **ci** — Skip flaky latency test in coverage
- **ci** — Use single-threaded tests for MSRV to avoid discovery topic race
- **discovery** — Detect active topics by checking path in addition to name
- **ci** — Skip additional flaky test_enhanced_scheduler in coverage
- **ci** — Skip flaky test_robot_performance_metrics in coverage
- **ci** — Fix coverage workflow crate exclusions
- **ci** — Exclude Linux-specific crates from macOS tests
- **ci** — Exclude horus_manager from MSRV tests due to linker issues
- Fix platform-specific test compilation issues
- **ci** — Exclude workspace-linkage-problematic crates from MSRV tests
- **ci** — Fix multi-platform CI builds
- **ci** — Fix benchmark timing.rs for non-x86_64 platforms and update CI
- **ci** — Update MSRV to 1.92.0 for horus_ai requirements
- **ci** — Fix multi-platform CI for macOS and update MSRV
- **ci** — Make security audit non-blocking
- **ci** — Make non-essential CI jobs non-blocking
- **ci** — Fix coverage and security audit failures
- Add missing org argument to publish method call
- Add clippy allow attributes for Rust 1.92 lints
- Suppress additional clippy warnings with allow attributes
- Fix clippy warnings for strict mode on main
- Remove/suppress unused imports, variables, and dead code in tests
- Remove unused mut and prefix unused variables in example
- Prefix unused seq2 variable with underscore
- Prefix unused pvalue variable with underscore
- Cast function to pointer before sighandler_t conversion
- DirectChannel mode only for POD types to prevent memory corruption
- Race condition in cross-process topic initialization (Issue #37)

### Documentation

- Fix more rustdoc warnings in horus_manager
- Fix rustdoc warnings in horus_manager
- Fix more rustdoc warnings
- Fix rustdoc warnings for strict mode

_192 further commit(s) in this range carried no conventional-commit prefix and are not summarized here._
## [0.1.8] — 2026-01-26

_No commits in this range used a conventional-commit prefix._

_13 further commit(s) in this range carried no conventional-commit prefix and are not summarized here._
## [0.1.7] — 2026-01-19

### Added

- complete and standardize keyboard keycodes

### Fixed

- fmt
- **docs** — add 142 missing MDX files that were gitignored
- **ci** — update GitHub Actions to v4 and fix CI workflow

_37 further commit(s) in this range carried no conventional-commit prefix and are not summarized here._
## [0.1.6] — 2025-12-02

_No commits in this range used a conventional-commit prefix._

_33 further commit(s) in this range carried no conventional-commit prefix and are not summarized here._
## [0.1.5] — 2025-11-16

### Added

- warn when license missing in horus.yaml

_23 further commit(s) in this range carried no conventional-commit prefix and are not summarized here._
## [alpha] — 2025-11-03

### Added

- add utility scripts for updates, recovery, and verification
- Replace .expect() with HorusResult in node constructors

### Fixed

- resolve CI failures - clippy lints and rustfmt
- **run** — dynamically find and link horus .rlib files for rustc
- **run** — correct rustc -L flag syntax
- **run** — add proper rustc dependency linking for horus crate
- Critical API consistency fixes for HORUS framework
- Standardize API error handling for consistency

### Changed

- Standardize Node trait error handling to use HorusResult
- Standardize error handling across codebase to use HorusResult and HorusError

### Documentation

- remove redundant git pull from utility scripts section
- update installation guide to recommend update.sh
- add cross-references to troubleshooting guide
- add comprehensive troubleshooting guide
- fix MDX compilation error in ai-integration.mdx
- comprehensive AI integration guide with local frameworks
- fix remaining ASCII diagram alignment in ai-integration.mdx
- fix ASCII diagram alignments across all pages
- fix c-bindings.mdx unsupported ::: syntax
- fix 'under development' warnings - make them specific
- **site** — Update website with latest benchmark results
- Update benchmark results with latest performance data
- Remove redundant HorusResult imports from documentation
- Standardize error handling to use HorusResult and HorusError

_173 further commit(s) in this range carried no conventional-commit prefix and are not summarized here._
