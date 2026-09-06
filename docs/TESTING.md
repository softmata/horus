# Testing HORUS

Where the tests are, which of them CI actually executes, and which are committed
and never run at all.

## Why this file exists

This workspace holds 251 integration-test targets and eleven loom models.
`CONTRIBUTING.md:42-94` covers the release rule, the two commands CI runs and
four ad-hoc invocations, and stops there. The gap that matters is not "where do
I add a test" — it is that a test can be committed, reviewed, merged and never
executed, and nothing about the tree tells you which. Five `horus_manager`
tests, twenty-six `horus_core` tests, three unit tests in
`horus_core/src/communication/topic/tests.rs`, a Puppeteer suite and six shell
scripts are in that state today. They are named below.

There is a second failure mode, subtler and just as common: a `--skip` or an
`--exclude` that names something which no longer exists. A skip is a substring
filter, so a renamed test silently stops being skipped and a deleted one
silently skips nothing. Four such entries are live in the workflows right now.
They are named too.

The document is written from the workflow files and from the sources, not from
intent. Where a source comment claims a suite is "wired into the docs-contract
workflow" and the grep says otherwise, the grep wins and the discrepancy is
recorded.

**What this file does not cover.** The C++ side has its own document,
`horus_cpp/TESTING.md` (last updated 2026-04-23), which indexes the gtest
binaries, the sanitizer jobs and the fuzz targets in far more detail than would
fit here; the one thing it omits is the layout-contract regeneration command,
which is in the last section below. Nothing here describes the Python binding
tests beyond where CI invokes them. This file makes no claim about what any test
*proves* — only about where it lives and whether it runs. And it does not claim
that a test which runs is a test that passes: several of the things below are
run and known to be red, and those are called out where the workflow says so.

## The test tree

Counted with `find <crate>/tests -maxdepth 1 -name '*.rs' -type f | wc -l`. Every
`.rs` file directly under a crate's `tests/` is its own cargo target;
subdirectories (`common/`, `helpers/`, `fixtures/`, `ui/`) are shared modules and
fixtures, not targets.

| Crate | Targets under `tests/` | Notes |
|:--|--:|:--|
| `horus_core` | 175 | the bulk of the suite; families below |
| `horus_manager` | 47 | CLI, scaffolding and docs-contract suites |
| `horus_net` | 14 | 13 auto-discovered plus `peer_process`, declared at `horus_net/Cargo.toml:27-30` with `harness = false` |
| `horus_sys` | 7 | five `parity_*`, plus `platform_intent` and `shm_holder_lifetime` |
| `horus` | 6 | prelude / macro / re-export contracts |
| `horus_cpp` | 1 | `proptest_ffi.rs`; the 24 `.cpp` files beside it are driven by `tests/CMakeLists.txt`, not cargo |
| `horus_cpp_macros` | 1 | `trybuild.rs`, over `tests/pass` and `tests/fail` |
| **Total** | **251** | |

`horus_types`, `horus_macros` and `benchmarks` have no `tests/` directory.
`horus_py` has one, but it holds no Rust: its coverage is a pytest suite of
sixteen `test_*.py` files plus `conftest.py`, 418 `def test_` functions as
measured today, run at `ci.yml:240-257`. The workflow's own comment there cites
413 functions and 442 parametrised cases (`ci.yml:236`), so that number has
drifted upward since it was written.

`peer_process` is worth knowing about before you meet it. It is a
`harness = false` helper that `horus_net/tests/cross_process.rs` locates by
scanning `deps/`, and `cargo test -p horus_net` runs it like any other target
with no arguments. It returns immediately on a bare or dash-prefixed invocation
(`horus_net/tests/helpers/peer_process.rs:79-98`) precisely so that a
contributor's first `cargo test` does not report a failure that is not one.

### The `horus_core` families

There is no written rule assigning a test to a file. The naming is a convention
the tree observes and nothing enforces. The rows overlap — `safety_monitor_intent.rs`
is counted under both `safety_*` and `*_intent` — so the column does not sum to
175 and is not meant to.

| Prefix / suffix | Files | What lives there |
|:--|--:|:--|
| `loom_*` | 10 | exhaustive interleaving models — see below |
| `action_*`, `actions_*` | 11 | the action server/client protocol |
| `rt_*` | 10 | real-time executor, deadlines, allocation guard |
| `*_intent` | 10 | "Level 7" behavioural-intent tests, which state the user-visible guarantee they protect (`topic_intent.rs:2-6`) |
| `service_*`, `services_*` | 10 | request/response services |
| `cross_process_*` | 9 | real child processes over SHM |
| `scheduler_*` | 8 | tick ordering, config, lifecycle |
| `safety_*` | 5 | e-stop, safety monitor, safety torture and battle suites |
| `determinism_*` | 5 | replayability and ordering |
| `log_*` | 4 | the logging subsystem across layers and processes |
| `stress_*` | 4 | `stress_rt_contention`, `stress_scheduling`, `stress_tensorpool_fragmentation`, `stress_topics` |
| `topic_*` | 4 | pub/sub surface |
| `chaos_*` | 3 | fault injection, mostly multi-process |
| `blackbox_*`, `record_replay_*`, `tensor*`, `watchdog_*` | 3 each | |
| `acceptance_*` | 2 | `acceptance_topic`, `acceptance_scheduler` — the two suites `CONTRIBUTING.md:89-90` names |

### Which suite a new test belongs in

The name is the smaller half of the decision. What actually decides where a test
goes is how CI reaches it:

1. **An ordinary test in an existing `tests/` file runs.** `integration-tests.yml:127`
   runs `--test '*'` across the workspace, so a new non-ignored test in any
   `tests/` target is executed on every PR, in release, without touching a
   workflow. `--test` accepts Unix glob patterns, which is why the single
   quoted `'*'` reaches every integration target of every selected package.
2. **A new file is also fine.** The glob picks it up. You do not need to register
   anything.
3. **A unit test in `src/` is reached only by a `--lib` run**, and the `--lib`
   runs carry long `--skip` lists. Check the lists before assuming yours runs;
   three unit tests are currently skipped by every one of them.
4. **An `#[ignore]`d test in a file that is not in the wired list runs
   nowhere.** `--test '*'` does not reach ignored tests
   (`integration-tests.yml:140-141`). Only the fifteen suite names hardcoded at
   `integration-tests.yml:222-226`, `:254-255` and `:293` are re-run with
   `--ignored`. If your test needs `#[ignore]` — because it re-execs the test
   binary as child processes, which is what these suites do
   (`integration-tests.yml:153-157`) — put it in a suite that is already on one
   of those lists, or add the suite name to the gating list in the same PR.
5. **A `horus_manager` test needs no wiring either.**
   `integration-tests.yml:794` runs `cargo test -p horus_manager --release`
   with no target selector, so every one of the 47 targets' non-ignored tests
   executes. An `#[ignore]`d one runs only if some workflow names it, and today
   only the `docs_*` suites are named — in `docs-contract.yml`, which is not a
   required check.

## What CI runs, exactly

Seven contexts are required on `main`, with strict mode on (verified against
`gh api repos/softmata/horus/branches/main/protection`, which returns
`"strict": true` and these seven):

`CI Success`, `Multi-Platform Success`, `Integration Tests Success`,
`Feature Matrix Success`, `Parity CI Success`, `All Distros Pass`,
`Run Benchmarks`.

Not required, and therefore not merge-blocking: `Docs Contract Success`,
`Monitor Tests Success`, `C++ Bindings CI Success`, `Distribution CI Success`,
the Safety (MIRI/ASan/TSan) jobs and coverage. `docs-contract.yml:503-506`
describes itself as "the single check branch protection points at"; branch
protection does not point at it. That is the one place in the repository where a
comment asserts a gate that does not exist, and it matters because
`docs-contract.yml` is the only workflow that runs any `docs_*` suite.

### The commands

Reproduce a CI failure in the configuration it happened in.

**`ci.yml` — `Test` job.** Unit tests only. `--lib` selects library targets, so
**not one of the 251 integration targets runs here** (`ci.yml:203-208`):

```bash
cargo test --workspace --exclude horus_py --lib --no-fail-fast -- \
  --test-threads=1 \
  --skip topic_cross_thread_1p_multi_c_spmc \
  --skip topic_cross_thread_mpmc_pre_initialized_99_percent \
  --skip topic_cross_thread_multi_p_multi_c_mpmc \
  --skip test_robotics_autonomous_car_perception
```

The job sets `HORUS_NAMESPACE: ci_${{ github.run_id }}` (`ci.yml:147`) and the
workflow sets `HORUS_SOURCE` to the checkout (`ci.yml:33`). The only `tests/`
targets `ci.yml` touches are the six loom models in its `loom` job
(`ci.yml:382-421`).

`ci.yml` also carries a `Test Quality Guard` job (`ci.yml:427-476`) that greps
the whole tree rather than running anything: it fails the build on
`is_ok() || is_err()` tautologies and on a bare `matches!(...);` statement whose
boolean is discarded, and warns on `let _ = format!` in test code. It is one of
the eight jobs `CI Success` requires (`ci.yml:481`), so those two patterns are
merge-blocking.

**`integration-tests.yml` — `Cross-Process IPC Tests` job.** This is where the
tree actually runs, and it is release-only (`integration-tests.yml:127-133`):

```bash
cargo test --workspace --exclude horus_py --release --test '*' -- \
  --test-threads=1 --nocapture \
  --skip shm_fanout_latency_percentiles_cross_thread \
  --skip stress_rt_isolation_from_compute_nodes \
  --skip stress_rt_1khz_under_cpu_contention \
  --skip stress_rt_1khz_baseline_no_contention \
  --skip stress_rt_multi_rate_under_contention
```

then the ten gating ignored suites (`integration-tests.yml:219-231`):

```bash
for suite in ipc_torture chaos_monkey message_type_matrix hardware_emulation \
             params_intent log_system_tests rust_python_full_matrix \
             rust_python_matrix humanoid_stress matrix_sweep; do
  cargo test -p horus_core --release --test "$suite" -- \
    --ignored --test-threads=1 --nocapture
done
```

then four more as advisory (`:244-264`), `multi_scheduler_matrix` as advisory on
its own (`:289-297`), and `horus_manager` (`:794`):

```bash
cargo test -p horus_manager --release -- --test-threads=1
```

**Everything else that runs a test, at a glance.**

| Workflow | Command | Reaches |
|:--|:--|:--|
| `ci.yml:203` | `--workspace --lib` | unit tests only |
| `ci.yml:220` | `-p horus_manager --lib source_resolver -- --ignored` | network resolver tests, `continue-on-error` at `:218` |
| `ci.yml:227,257` | `-p horus_py --no-default-features`; then pytest | the binding layer |
| `ci.yml:383-419` | `-p horus_core --test loom_<six>` | six of the ten loom models, in debug, with `RUST_LOG=loom=debug` |
| `integration-tests.yml:127,228,257,293,794,1178+` | `--test '*'` + named suites | the tree, in release |
| `multi-platform.yml:73,112` | `-p horus_types -p horus_macros -p horus_cpp_macros` | portable crates on macOS x86-64 and ARM64 |
| `multi-platform.yml:154,204` | `--no-default-features -p horus_sys --lib`, then `-p horus_core --lib` | Windows, default-features off |
| `multi-platform.yml:390` | `--workspace --lib` minus `horus`, `horus_library`, `horus_py`, `horus_manager` | the MSRV job — see the note below |
| `cross-platform-parity.yml:69,110,171` | `-p horus_sys --test parity_shm --test parity_process --test parity_discover --test parity_fs --test parity_platform` | five of the seven `horus_sys` targets, on Linux, macOS and Windows |
| `cross-platform-parity.yml:140` | `-p horus_manager --test install_contract --test completion_install_contract` | the installer's shell, on macOS's bash 3.2 (`:130-136`) |
| `docker-distro-tests.yml:50-61` | the image `CMD`, `cargo test --no-default-features -p horus_sys -- --test-threads=1`, then `tests/docker/test_user_workflow.sh` | ubuntu, debian, fedora, alpine, arch — no target selector, so `horus_sys` doctests run here |
| `docs-contract.yml:138-167` | `-p horus_manager --test docs_examples/docs_examples_cpp/docs_examples_python/docs_contract/docs_scaffold/docs_behavior` | tier 1, hermetic, every PR |
| `docs-contract.yml:220-250` | the same targets plus `docs_cli_contract` and `docs_manifest`, with `HORUS_DOCS_DIR` set | tier 2, against a pinned `horus-docs` checkout |
| `docs-contract.yml:334` | `-p horus_manager --test examples_contract` with `HORUS_TEST_BUILD_EXAMPLES=1` | builds every shipped example, every PR |
| `monitor-tests.yml:95,104,154` | `-p horus_manager --lib`, `--test harness_tests`, `--test uat_workflows` | path-filtered to `horus_manager/**`, `horus_core/**` |
| `monitor-tests.yml:195` | `--test discovery_stress` | `workflow_dispatch` only (`:162`), and the workflow records that it has known failing tests (`:190-193`) |
| `cpp-bindings.yml:82,88,95` | `-p horus_cpp` (no selector), `--test proptest_ffi`, and the `loom_scheduler` model | path-filtered; not required |
| `safety.yml:92,146,211` | MIRI, ASan, TSan over `--lib` | weekly, plus PRs touching `horus_core/src/memory/**` or `communication/**` |
| `coverage.yml:112` | `cargo llvm-cov --workspace` | not required; `continue-on-error: true` at `:33` |
| `feature-matrix.yml:71,111-115,150` | `maturin build` and `cargo check` only | **no `cargo test` at all** |
| `benchmarks.yml:182-186` | `cargo run --release --bin cross_process_benchmark` / `robotics_messages_benchmark` | benchmark binaries; the gate greps their output for `✗ FAIL` (`:200-213`) |

`Feature Matrix Success` and `Run Benchmarks` are required contexts that execute
no `cargo test`.

**The MSRV job does not test the declared MSRV.** `Cargo.toml:28` declares
`rust-version = "1.90"` for the whole workspace, and every member inherits it
with `rust-version.workspace = true`. `multi-platform.yml:350` pins the MSRV
matrix to `"1.92.0"`, and `multi-platform.yml:26` sets an `MSRV: "1.92.0"` env
var that nothing reads — `env.MSRV` appears nowhere. So the floor the manifest
publishes is two minor versions below the floor CI verifies, and a crate
feature introduced in 1.91 or 1.92 would pass every check while making the
declared MSRV false. Either number can be the right one; they cannot both be.

## Release versus debug

`CONTRIBUTING.md:46-52` states the rule and the reason: a debug build is roughly
an order of magnitude slower, so a 1 kHz RT test, a latency percentile or a
stress soak can fail in debug purely because the code cannot keep up. It names
seven tests across `stress_rt_contention`, `scheduler_config_test`,
`production_fanout_battle` and `safety_battle_tests` that fail in debug and pass
in release. That attribution is `CONTRIBUTING.md`'s; it is not re-verified here.

The operational consequence is sharper than the rule suggests: **the only
workflow that runs the `tests/` tree runs it exclusively in release**
(`integration-tests.yml:89` builds `--release --tests`, `:127` tests with
`--release`). There is no debug run of the integration suite anywhere in CI. If
you run `cargo test --test <suite>` locally without `--release` and it fails, the
first thing to try is `--release`.

## `#[ignore]` and how to run those tests

`#[ignore]` is used here for two different things, and telling them apart is the
difference between a test that runs and one that does not.

**Mechanism, not neglect.** The cross-process suites re-exec their own test
binary as child processes — `chaos_cross_process.rs:36` spawns
`<test> --exact --nocapture --ignored` per role, selecting the child through the
`HORUS_CHAOS_XP_CHILD` variable (`:20`) — so publisher and subscriber are
genuinely separate processes. The `#[ignore]` is how the child is addressed.
`loom_pod_broadcast`'s `naive` variant is the same idea inverted: it is ignored
because it is *expected to fail*.

**Counts, measured with `grep -cE '^\s*#\[ignore' horus_core/tests/*.rs`:**

| | Count |
|:--|--:|
| `#[ignore]` attributes in `horus_core/tests` | 85 |
| …re-run with `--ignored` as a gate (`integration-tests.yml:222-226`) | 36 |
| …re-run with `--ignored` as advisory (`:254-255`, `:293`) | 19 |
| …reached by nothing | 30 |
| Same total, in `horus_manager/tests` | 27 |

An unanchored `grep -c '#\[ignore'` returns 93 because it also matches prose
inside module doc comments; the workflow records that an earlier draft quoted
that number (`integration-tests.yml:148-151`). Of the 85, five carry a real
reason string (`#[ignore = "…"]`), four more carry a trailing `//` comment, and
76 are a bare `#[ignore]` with nothing at all — which is the figure
`integration-tests.yml:143-144` quotes.

The 30 that nothing reaches, and what each one actually is:

- `cli_runtime_integration` (18), `launch_and_workflow` (5) and
  `real_project_ipc` (3) resolve `target/debug/horus` and
  `target/debug/examples/<name>` by hardcoded path
  (`cli_runtime_integration.rs:20,27`), so they need a debug build of
  `-p horus --examples` plus `horus_manager` that no job produces
  (`integration-tests.yml:160-166`). This is the real dead weight: 26 tests.
- `participant_count_leak.rs:62-63`,
  `transient_subscribers_permanently_flip_a_topic_to_a_lossy_backend`, whose
  reason string reads "documents an open defect: see the module comment". An
  open defect recorded in a test nothing runs is a defect nobody is told about.
- `loom_pod_broadcast`'s `naive` variant — see below; it is supposed to fail.
- `acceptance_topic.rs:221-222`, `test_scenario_high_frequency_publishing`,
  marked "Flaky on CI".
- `crash_recovery.rs:115-116`, `panic_hook_subprocess_helper`. This one is **not**
  dead: its reason string says it is a subprocess entry point invoked by
  `test_panic_hook_writes_crash_report` with `HORUS_CRASH_TEST=1`, and that
  parent test is not ignored, so it runs in the release sweep and drives the
  helper as a child. Counting it as missing coverage would be wrong.

In `horus_manager/tests` the 27 break down as `docs_contract` 11,
`docs_manifest` 6, `docs_parity` 3, `docs_examples_cpp` 2,
`docs_examples_python` 2, `docs_examples` 1, `cpp_toolchain_contract` 1,
`readme_contract` 1. `docs-contract.yml` reaches all of them except the last
three.

To run ignored tests:

```bash
# every ignored test in one suite
cargo test -p horus_core --release --test chaos_cross_process -- --ignored --test-threads=1 --nocapture

# ignored and non-ignored together
cargo test -p horus_core --release --test ipc_torture -- --include-ignored --test-threads=1

# one test by name
cargo test -p horus_core --release --test rt_deadline_enforcement -- \
  test_budget_violation_node_still_runs --exact --nocapture
```

Use `--include-ignored`, not `--ignored`, when a suite mixes both — the
docs-contract job makes exactly this distinction at `docs-contract.yml:211-220`,
and guards it with a test (`the_live_docs_job_runs_the_tests_this_file_ignores`)
that fails if a step ever loses the flag.

## The loom models

Ten under `horus_core/tests/`, one in a separate crate. They use `loom`'s
instrumented atomics directly (`loom = "0.7"` at `horus_core/Cargo.toml:69`), so
they need no `--cfg loom` — except `horus_cpp/loom_tests`, which does, and which
is excluded from the parent workspace because loom compilation conflicts with
tokio's `cfg(not(loom))` gates (`horus_cpp/loom_tests/Cargo.toml:7-11`). That
job sets `RUSTFLAGS: --cfg loom` and `LOOM_MAX_PREEMPTIONS: "3"`
(`cpp-bindings.yml:96-98`).

Because all ten `horus_core` models are ordinary `#[test]` functions in ordinary
`tests/` targets with no `cfg` gate, **all ten are selected by
`integration-tests.yml:127`'s `--test '*'` sweep** and run in release on every
PR. Six of them additionally get a named step in `ci.yml`'s `loom` job. The
difference between the two groups is not "runs" versus "does not run" — it is
whether a failure arrives as a named step or as one line inside a 45-minute
workspace sweep, and whether the model is also exercised in debug.

| Model | Protocol modelled | Named step |
|:--|:--|:--|
| `loom_fanout` | the per-slot-versioned seqlock `ShmFanoutRing` uses, including the producer overwriting the slot mid-copy | `ci.yml:383` |
| `loom_pod_broadcast` | `PodShm` broadcast, the default backend for POD types, whose producer has no backpressure | `ci.yml:392` |
| `loom_concurrency` | watchdog heartbeat / PRNG / deterministic clock | `ci.yml:397` |
| `loom_migration` | the migration control plane: CAS lock, drain, epoch bump, mode switch | `ci.yml:402` |
| `loom_migration_data_plane` | a cached consumer position rewritten by a migration mid-drain — the combination TSan flags | `ci.yml:414` |
| `loom_participant` | participant slot claim, `active` 0→2→1, lease reclaim, role counters | `ci.yml:419` |
| `loom_mp_claim` | the CAS slot-claim that replaced an overshooting `fetch_add` in `send_shm_mp_pod`/`_serde` | none |
| `loom_sp_mp_flag` | the SpscShm→MpscShm ready-flag conversion; without it, messages buffered under SpscShm become permanently unreadable after the migration | none |
| `loom_spmc_epoch_flush` | `handle_epoch_change` publishing a frontier through `header.tail`, which on SpmcShm is a shared claim cursor and is flushed without `fetch_max` | none |
| `loom_tensor_pool` | the TensorPool Treiber free-stack: no double-allocation, no `next_free` cycle | none |
| `loom_scheduler` (`horus_cpp/loom_tests/`) | the scheduler/callback atomic contract, modelled abstractly | `cpp-bindings.yml:95` — path-filtered, not required |

**Four of the ten are invisible in the workflow files.** Grepping
`.github/workflows/` for each name: `loom_mp_claim` appears once, in a *comment*
at `ci.yml:406`; `loom_sp_mp_flag`, `loom_spmc_epoch_flush` and
`loom_tensor_pool` appear nowhere. Each models a hazard its file header ties to a
specific past bug — the MPSC overshoot, the sp→mp flag gap, the SpmcShm
re-delivery, the TensorPool double-allocation. They are gating merges, through
`Integration Tests Success`, but nobody reading the CI configuration would know
they exist. Run them directly when touching the send paths, the migration path
or the tensor pool:

```bash
cargo test -p horus_core --test loom_mp_claim -- --nocapture
cargo test -p horus_core --test loom_sp_mp_flag -- --nocapture
cargo test -p horus_core --test loom_spmc_epoch_flush -- --nocapture
cargo test -p horus_core --test loom_tensor_pool -- --nocapture
```

**`loom_pod_broadcast`'s `naive` variant must keep failing.**
`naive_protocol_tears_proving_the_model_can_detect_it`
(`loom_pod_broadcast.rs:223-262`) re-implements the pre-fix protocol and asserts
the same invariant the real model does. The assertion is *supposed* to trip: it
is what proves the model is not passing vacuously. It is `#[ignore]`d so the
suite stays green, and `loom_pod_broadcast` is on the list of suites
deliberately left out of the `--ignored` re-runs (`integration-tests.yml:186`),
so nothing runs it. If you change the seqlock stamping, run it and confirm it
still fails:

```bash
cargo test -p horus_core --test loom_pod_broadcast -- --ignored naive
```

A `naive` variant that starts *passing* means the model has stopped being able
to see the bug, and the green run of the real test above it means nothing.
Whether the four unnamed models pass today has not been checked here; the claim
is about where they are invoked, not about their colour.

## The `test-utils` feature

`horus_core/Cargo.toml:92` declares `test-utils` with no dependencies. It gates
`horus_core::testing`, which is compiled under
`#[cfg(any(test, feature = "test-utils"))]` and `#[doc(hidden)]`
(`horus_core/src/lib.rs:49-52`).

What it exports (`horus_core/src/testing/mod.rs:10-12`):

| Item | Module |
|:--|:--|
| `MockTopic`, `MockTopicConfig` | `mock_topic.rs` |
| `ShmFault`, `ShmFaultInjector`, `NetworkFault`, `MockNetworkEndpoint` | `shm_fault.rs` |
| `TestNode`, `TestNodeBuilder` | `test_node.rs` |
| `test_spawn` | `mod.rs:23` — `std::thread::spawn` with a 512 KB stack, so a suite running many multi-threaded tests in parallel does not exhaust memory (`mod.rs:14-22`) |

One crate in the workspace turns it on: `horus_cpp`, as a dev-dependency
(`horus_cpp/Cargo.toml:35`). A grep for `test-utils` across `horus` matches only
that line, the feature declaration, the `cfg` in `lib.rs` and the module's own
doc comment; it appears in no markdown file in `horus`, and neither the feature
nor `MockTopic`, `ShmFaultInjector` or `TestNodeBuilder` appears anywhere under
`horus-docs/content/`. If you are writing a downstream test that needs to drive
an error path without real shared memory, this is the thing to reach for:

```toml
[dev-dependencies]
horus_core = { path = "../horus_core", features = ["test-utils"] }
```

Because the module is `#[doc(hidden)]`, it does not appear in rendered rustdoc.
Read it with `cargo doc -p horus_core --features test-utils --document-private-items`,
or read the three files directly.

## What CI deliberately skips

Every `--skip` in the workflows, and the reason where a workflow gives one.
There is no single set of skipped tests: the lists differ per workflow —
`ci.yml` 4, `integration-tests.yml` 5, `safety.yml` 7 for ASan and 12 for TSan,
`coverage.yml` 19 including one substring, `multi-platform.yml` 5 on Windows and
4 on MSRV.

| Test | Skipped in | Stated reason |
|:--|:--|:--|
| `stress_rt_1khz_baseline_no_contention`, `stress_rt_1khz_under_cpu_contention`, `stress_rt_multi_rate_under_contention`, `stress_rt_isolation_from_compute_nodes` | `integration-tests.yml:130-133`, `coverage.yml:118` | they assert RT timing while eight compute nodes saturate the box; on a shared 2-vCPU runner that measures the runner, not HORUS — the job saw 34.24% spread (`integration-tests.yml:106-112`) |
| `shm_fanout_latency_percentiles_cross_thread` | `integration-tests.yml:129`, `coverage.yml:118` | latency percentiles are a performance gate, not an IPC correctness check; the 1P4S case intermittently leaves one subscriber unscheduled (`:102-104`) |
| `topic_cross_thread_1p_multi_c_spmc`, `topic_cross_thread_mpmc_pre_initialized_99_percent`, `topic_cross_thread_multi_p_multi_c_mpmc` | `ci.yml:205-207`, `safety.yml:152-154` and `:222-224`, `coverage.yml:118`, `multi-platform.yml:204` and `:394-396`, `integration-tests.yml:1228-1230` | seven places. Only `multi-platform.yml:156-159` gives a reason — throughput thresholds that are a property of the runner. See the note below: these three now run nowhere at all |
| `test_robotics_autonomous_car_perception` | `ci.yml:208`, `safety.yml:151` and `:220`, `coverage.yml:118`, `multi-platform.yml:397` | none stated. It survives only in `multi-platform.yml:204`'s Windows job, which does not skip it |
| `test_high_performance_config`, `test_auto_derive_tick_rate_from_fastest_node` | `coverage.yml:118` | both assert a tick *count* as a proxy for a rate; llvm-cov instrumentation makes the loop miss its rate, so the count measures the instrumentation — 1 tick against a threshold of 10 (`coverage.yml:104-111`). Both keep their teeth in `integration-tests.yml`'s release sweep, which the comment says and which is true: they live in `horus_core/tests/scheduler_config_test.rs` |
| `a_clone_growing_the_mapping_does_not_strand_its_siblings` | `safety.yml:225` (TSan only) | `Topic::send` is `send_lossy`; TSan sees the deliberate overwrite race and cannot see that the downstream stamp re-validation is what makes the read sound (`safety.yml:200-210`) |
| `migrate_pod_shm_to_spsc_shm` | `safety.yml:217` (TSan only) | named in the comment above as covered by the same reasoning |
| `test_robotics_drone_fast_imu_slow_camera`, `test_tick_hz_very_large` | `safety.yml:219`, `:221` (TSan only) | none stated; the comment at `:209-210` names `migrate_pod_shm_to_spsc_shm` and the cross-thread tests, not these |
| `benchmark_ffi_send_overhead`, `large_graph_performance`, `test_recording_overhead_benchmark` | `safety.yml:148-150`, `:214-216` | timing assertions under `-Zsanitizer` instrumentation |
| any test whose name contains `xproc_` | `coverage.yml:118` | a substring skip; it removes all 18 `xproc_*` tests, every one of them in `horus_net/tests/cross_process.rs`, from the coverage run |
| `test_robot_performance_metrics`, `test_enhanced_scheduler`, `test_custom_exotic_robot_config`, `hlog_latency_under_contention_4_threads`, `hlog_latency_p99_under_50us`, `dual_write_overhead_acceptable` | `coverage.yml:118` | "flaky due to shared memory state or timing" (`coverage.yml:101`) |
| `mp_send_no_overshoot_corruption` | `multi-platform.yml:204` (Windows only) | **an open data-loss defect, fully diagnosed in place** (`multi-platform.yml:170-200`): the consumer drained 1798/1800 while producers published 1800/1800, with no torn or aliased delivery, on `try_send`, which is the API that is supposed to refuse rather than overwrite. Ruled out: drain budget, producers abandoning, Linux preemption. Does not reproduce off Windows |
| `multi_scheduler_matrix` (whole suite) | `integration-tests.yml:289-297` | **advisory, not skipped.** `deterministic_alongside_normal_scheduler` dies with SIGBUS in `dispatch::send_shm_mp_pod::<CmdVel>` → `Scheduler::tick_once`, reproduced 2 runs in 5 on a 12-core box on main. Kept running and kept separate so the day it stops dying is visible |
| `chaos_cross_process`, `production_validation`, `chaos_xp_schedulers`, `production_fanout_battle` | `integration-tests.yml:244-264` | **advisory.** Two vCPUs cannot schedule five or ten real child processes to their timing assumptions. To promote one back to gating: show it green across ~20 consecutive 2-vCPU runs, or move it to dedicated hardware |

### Four skips that no longer name anything

`--skip` is a substring filter over test names. A skip that matches nothing is
silent, in both directions: the workflow implies a test is excluded when it is
not, or excludes a test that no longer exists and hides the fact that the
coverage went with it.

- **`multithread_nonpod_subscribers_each_get_full_stream`** is skipped at
  `coverage.yml:118` and `multi-platform.yml:204`. No such test exists. The name
  survives only in a comment at
  `horus_core/tests/lapped_consumer_is_countable.rs:9`; the live test is
  `multithread_nonpod_subscribers_each_get_a_contiguous_accounted_stream`
  (`horus_core/src/communication/topic/tests.rs:6551`), which the old string is
  not a substring of. So both skips are inert, and the twelve-line comment at
  `multi-platform.yml:161-168` — which records that the non-POD multi-subscriber
  broadcast path "appears not to work on this platform at all", and that before
  the barrier fix it hung the Windows job rather than failing it — is attached to
  a skip that has not applied since the rename.
- **`test_watchdog_graduated_warning`** is skipped at `safety.yml:218`. It does
  not exist anywhere in the repository.
- **`test_direct_channel_latency_estimation`** is described at
  `coverage.yml:102` as "flaky due to coverage instrumentation affecting
  timing". It does not exist, and it is not in the skip list either.
- **`--exclude horus_library`** appears at `coverage.yml:115` and
  `multi-platform.yml:391`, and `horus_library/**` is still a path filter at
  `cpp-bindings.yml:15,22` and `monitor-tests.yml:17,25`. That crate was
  decomposed into the external `horus-robotics` and `horus-tf` repositories
  (`ci.yml:423-425`, `safety.yml:97-99`) and appears in no `Cargo.toml`. The
  exclusions are harmless; the path filters mean those two workflows advertise a
  trigger that can never fire.

### Three unit tests that run nowhere

`topic_cross_thread_1p_multi_c_spmc`,
`topic_cross_thread_mpmc_pre_initialized_99_percent` and
`topic_cross_thread_multi_p_multi_c_mpmc` live in
`horus_core/src/communication/topic/tests.rs` (`:576`, `:2094`, `:762`). They are
unit tests, so the only runs that can reach them are the ones that build
`horus_core`'s library test target — and every one of those skips them by name:
`ci.yml:203`, `safety.yml:146` and `:211`, `coverage.yml:112` (via the skip list
at `:118`), `multi-platform.yml:204` and `:390`, and
`integration-tests.yml:1226`. The two remaining candidates are filtered to other
modules: `safety.yml:92`'s MIRI run selects `memory::simd`,
`memory::rt_allocator` and `memory::platform`, and `integration-tests.yml:1178`
names a single scheduler test with `--exact`.

Three cross-thread topic tests — the SPMC and MPMC paths, and the
pre-initialised 99% case — are therefore executed by nothing on any platform.
The rationale given anywhere is `multi-platform.yml:156-159`, which is about the
Windows runner being slow. That reason does not extend to the six other places
the skip was copied to.

### The stress_rt case

Those four plus `shm_fanout_latency_percentiles_cross_thread` **run nowhere in
CI**, and the workflow says so plainly (`integration-tests.yml:114`). An earlier
version of that comment claimed they were "exercised by the dedicated
benchmark/stress workflows"; they are not — `benchmarks.yml` runs the binaries in
`benchmarks/` and gates on the real-time-suitability lines they print
(`benchmarks.yml:182-186`, `:200-213`), and never invokes these cargo tests. All
five live under `horus_core/tests/` (`stress_rt_contention.rs:402,490,548,670`
and `production_fanout_battle.rs:158`), so `integration-tests.yml:127` and
`coverage.yml:112` are the only invocations that could reach them, and both skip
them by name. They are developer-run gates. Anyone changing the tick path should
run:

```bash
cargo test -p horus_core --release --test stress_rt_contention -- --nocapture
```

The gate-logic tests in that same file do run, because they carry no timing
dependency: `spread_gate_alone_cannot_see_a_single_long_stall` (`:808`),
`stall_rate_catches_degradation_below_the_tail_bound` (`:851`),
`spread_gate_still_fires_on_broadly_noisy_timing` (`:879`),
`clean_timing_passes_every_limit` (`:904`) and
`rate_floor_catches_a_node_running_at_the_wrong_rate` (`:927`). None carries
`#[ignore]`. The workflow's own note (`integration-tests.yml:124-126`) lists
three of the four families and omits `rate_floor_*`.

## Committed and never executed

The most useful list in this document. Each of these is checked in, looks like
coverage, and gates nothing.

| Thing | Where | Status |
|:--|:--|:--|
| `cli_runtime_integration` (18 ignored), `launch_and_workflow` (5), `real_project_ipc` (3) | `horus_core/tests/` | need a debug build of `-p horus --examples` plus `horus_manager` that no job produces (`integration-tests.yml:160-166`) |
| `transient_subscribers_permanently_flip_a_topic_to_a_lossy_backend` | `participant_count_leak.rs:62` | `#[ignore]`d with the reason "documents an open defect" |
| `naive_protocol_tears_proving_the_model_can_detect_it` | `loom_pod_broadcast.rs:225` | deliberately excluded (`integration-tests.yml:186`); it is supposed to fail, so running it in CI would be wrong — but nothing checks that it *still* fails |
| `docs_parity` | `horus_manager/tests/docs_parity.rs` | all three tests are `#[ignore]`d with the reason "needs a horus-docs checkout; wired into the docs-contract workflow" (`:193,:259,:336`) — and it **is** wired: `docs-contract.yml` runs `cargo test -p horus_manager --test docs_parity -- --ignored --nocapture`. This row previously read "`grep -rn docs_parity .github/` returns nothing … never runs", which was true when written and was falsified by the commit that added the invocation |
| `readme_contract`'s `#[ignore]`d test | `readme_contract.rs:146` | reason: "slow: builds a scratch crate; run in the docs-contract job". That job **does** name the target now (`docs-contract.yml`, `--include-ignored`); this row previously said it did not. The other 10 tests in the file also run, via `integration-tests.yml:794` |
| `cpp_toolchain_contract`'s `#[ignore]`d test | `cpp_toolchain_contract.rs:464` | reason: "slow: cross-compiles horus_cpp; run with `--ignored`". Nothing does. The other 19 run |
| `.config/nextest.toml` | repo root config | a fully worked per-test timeout guard (60 s × 3 hard kill at `:50`, `test-threads = 1` at `:66`, `retries = 0` at `:70`, `leak-timeout = "500ms"` at `:77`, a `ci` profile with JUnit output at `:83-101` and a `soak` profile at `:103-110`) that no workflow uses — `grep -rn nextest .github/` returns nothing, and no markdown mentions it |
| `horus_manager/tests/web_e2e/` | Puppeteer suite, `package.json:5-7` | referenced only by `.github/dependabot.yml:105`, whose own comment records that no CI workflow runs the suite and that puppeteer is `hasInstallScript: true`, so the dependency graph is exercised exclusively on workstations |
| `tests/cli_runtime_test.sh`, `tests/test_horus_cli.sh`, `tests/test_horus_integration.sh`, `tests/valgrind_dlpack.sh` | repo `tests/` | named in no workflow |
| `tests/qa/` (`launch/` with eight YAML scenarios, `pubsub_rust/`, `pubsub_python/`) | repo `tests/` | `CONTRIBUTING.md:94` calls them end-to-end QA scenarios; nothing runs them |
| `tests/docker/run_all.sh` | repo `tests/docker/` | `docker-distro-tests.yml:43-61` builds the five Dockerfiles and runs the image `CMD` and `test_user_workflow.sh` directly; `run_all.sh` is unreferenced. `acceptance_linux_macos.sh` and `acceptance_windows.ps1` beside it *are* used, by `cross-platform-parity.yml` and `distribution.yml` |
| `horus_core`'s doctests | 13 blocks tagged ```` ```rust ```` and one ```` ```no_run ```` in `horus_core/src` | see below |

`cpp_toolchain_contract` carries an `#[ignore]` *reason string asserting it is
wired* while nothing runs it, and a reader of the source has no way to discover
that short of grepping the workflows. If you add an `#[ignore]` whose reason
names a workflow, add the invocation in the same commit.

`docs_parity` and `readme_contract` used to be listed here for the same fault
and have since been wired up — which exposed the mirror-image hazard this table
now demonstrates twice over. **A table asserting that something is unrun is
itself untested prose, and it rots in the direction that wastes a newcomer's
time**: it sends them to hand-run a suite CI already covers, and it teaches them
to distrust the workflows. Re-derive the rows against
`grep -rn '<target>' .github/` before relying on any of them.

**On `nextest.toml`.** Its stated premise has partly expired: it says at `:8`
that "None of the jobs in `.github/workflows/ci.yml` set `timeout-minutes`", and
all nine of them now do (`ci.yml:42,83,98,145,264,311,352,430,483`).
`integration-tests.yml` sets ten step-level timeouts and `coverage.yml` caps
itself at 120 minutes. But `safety.yml`, `multi-platform.yml` and
`cross-platform-parity.yml` contain the string `timeout-minutes` zero times, so a
hung test in any of them still runs to GitHub's 360-minute default with no test
name attached — and two of those three produce required contexts. The config
file was written to close exactly that, and it is wired to nothing.

**On the doctests.** Every `horus_core` invocation in `.github/workflows` carries
`--lib` or `--test <name>`, and both selectors exclude doctests; `cargo test
--all-targets` is documented as excluding them too, and `ci.yml:345`'s
`cargo doc --workspace --no-deps` builds documentation without running any. The
one invocation with no target selector is `coverage.yml:112`
(`cargo llvm-cov --workspace`), which is not a required context and which does
not pass `--doctests`, the flag `cargo-llvm-cov 0.9.0` describes as "Including
doc tests (unstable)". Whether cargo-llvm-cov nonetheless executes doctests
without that flag is a property of the tool and is **not verified here**; what is
verified is that no invocation asks for them. A further 146 blocks are
`rust,ignore` and 17 are `ignore`, so rustdoc never compiles those by design, and
every code fence in `horus_core/src` carries a language tag — an untagged ```` ``` ````
opener would be compiled as Rust, and there are none.

By contrast `horus_sys`'s doctests do run, on five distributions: the container
`CMD` is `cargo test --no-default-features -p horus_sys -- --test-threads=1`
(`tests/docker/Dockerfile.ubuntu:24`; all five Dockerfiles carry the same line) with no target selector, and
`All Distros Pass` is required.

## The local environment a test run needs

| Variable | Set by CI at | What it does |
|:--|:--|:--|
| `HORUS_SOURCE` | `ci.yml:33`, `integration-tests.yml:34`, `safety.yml:35`, `coverage.yml:24`, `docs-contract.yml:301` | `horus_manager`'s `cargo_gen`/`cmake_gen`/`new`/`pkg` tests generate `.horus` build files and need the HORUS tree. `find_horus_source_dir()` probes `HORUS_SOURCE`, then `/horus`, `~/softmata/horus`, `~/horus`, `/opt/horus`, `/usr/local/horus`, then the installer cache (`horus_manager/src/commands/run/run_rust.rs:1046-1095`). Each candidate must contain `horus/Cargo.toml`, so a checkout at one of those paths needs no variable |
| `HORUS_NAMESPACE` | `ci.yml:147`, `integration-tests.yml:42`, `safety.yml:33`, `coverage.yml:40`, `monitor-tests.yml:39`, `benchmarks.yml:73`, `cpp-bindings.yml:39`, and per-job in `multi-platform.yml` and `cross-platform-parity.yml` | isolates the SHM namespace so concurrent CI runs on one machine do not collide. **You usually do not need it locally**: `generate_namespace()` gives a test binary its own namespace derived from the cargo target directory, so a `cargo test` run is already isolated from a running robot (`horus_sys/src/shm/mod.rs:177-207`, `:70-89`) |
| `HORUS_DOCS_DIR` | `docs-contract.yml:219,226,234,241,249` (tier 2) and `:390,398,498` (the nightly jobs) | points the nine `horus_manager` suites that read the docs (`docs_contract`, `docs_cli_contract`, `docs_examples`, `docs_examples_cpp`, `docs_examples_python`, `docs_manifest`, `docs_parity`, `docs_scaffold`, `readme_contract`) at a `horus-docs` checkout. Without it the doc-derived tests report a skip rather than passing silently (`docs_scaffold.rs:26-28`) |
| `HORUS_TEST_BUILD_EXAMPLES` | `docs-contract.yml:333` | turns on the half of `examples_contract` that actually runs `horus build` over every shipped example. It was set nowhere until recently, and fifteen of the sixteen shipped examples were broken while CI stayed green (`docs-contract.yml:263-268`) |

CI creates the SHM tree before every test job — five subdirectories at mode 0700
(`ci.yml:176-186`, repeated verbatim in `safety.yml`, `coverage.yml`,
`integration-tests.yml`, `monitor-tests.yml`, `cpp-bindings.yml` and
`multi-platform.yml`):

```bash
mkdir -p /dev/shm/horus_<namespace>/{topics,nodes,control,network,scheduler}
chmod -R 700 /dev/shm/horus_<namespace>
```

The base path is `/dev/shm/horus_<namespace>` on Linux, `/tmp/horus_<namespace>`
on macOS and `%TEMP%\horus_<namespace>` on Windows
(`horus_sys/src/shm/mod.rs:242-269`). Do not `sudo mkdir` these: root-owned
directories are what made `SchedulerRegistry::open`
(`horus_core/src/scheduling/registry.rs:158`) and the presence writer fail with
`PermissionDenied`, and `chmod 777` breaks
`regression_shm_topics_dir_permissions_restricted`
(`horus_core/tests/regressions.rs:307`), which asserts the topics directory is
owner-only (`ci.yml:180-184`).

**Run serially.** `--test-threads=1` is not a preference. Integration tests share
the process-global SHM namespace, and `cleanup_stale_shm()`
(`horus_core/tests/common/mod.rs:104-119`) `remove_dir_all`s the whole topics and
nodes directories. The guard it returns serialises the tests that call it and is
reentrant, but some suites share process-global state — service registries, RT
CPU timing, cross-process SHM — without calling it and still race
(`common/mod.rs:35-37`). Bind the guard for the whole test; it is `#[must_use]`
with a message saying why (`common/mod.rs:68-70`).

Use `unique(prefix)` for topic names (`common/mod.rs:126`) and
`test_pool_id(POOL_BASE_*)` for tensor-pool ids (`:315`). The pid-spread family
owns the band 20 000–20 599, one hundred ids per base, six bases
(`common/mod.rs:253-311`), and
`tensor_pool_concurrent::pool_id_ranges_cannot_overlap`
(`horus_core/tests/tensor_pool_concurrent.rs:271`) asserts the band is exactly
consumed, so adding a base without widening `POOL_BAND_END` fails. Do not pick an
id in 9500–10200: that range is densely occupied by fixed ids in
`horus_core/src/memory/tensor_pool.rs`'s own unit tests, `tests/regressions.rs`
and `benchmarks/benches/tensor_pool.rs` (`common/mod.rs:243-249`), and
`common/mod.rs:19-23` records that this note used to recommend exactly the range
those tests already hold.

## Regenerating the C++ layout contract

`horus_cpp/include/horus/layout_contract.hpp` pins every field offset of every
type reachable through `impl_pod_topic_c_api!`. It exists because the C ABI
`_send`/`_recv` entry points are raw `ptr::read`/`ptr::write` through a
`*mut c_void` and nothing checked that the C++ struct matched: the 2026-07-30
audit found `JointCommand` at 928 bytes in Rust and 88 in the C++ header
(an 840-byte overrun), `AudioFrame` overrunning by 11040, and `ServoCommand` the
same 24 bytes on both sides with entirely different fields — which no `sizeof`
sweep can catch (`horus_cpp/src/layout_contract.rs:3-26`).

`contract_file_is_current` (`layout_contract.rs:214`) fails when a Rust message
type changes shape. It is a unit test in `src`, so it runs under every `--lib`
invocation, including `ci.yml:203`, and under `cpp-bindings.yml:82`. The command
that fixes it is:

```bash
cargo test -p horus_cpp --no-default-features regenerate_layout_contract -- --ignored --nocapture
```

That string appears four times, all of them in Rust source or in the file it
generates: the module doc comment (`layout_contract.rs:38`), the header the
generator emits (`:169`, which is why it also lands in
`horus_cpp/include/horus/layout_contract.hpp:5`), and the assertion message the
gate prints when it fails (`:221`). It appears in no markdown file in the
repository — `horus_cpp/TESTING.md` included — so a reader who has not hit the
failure has no way to find it except by reading the source.

Then reconcile the C++ headers with the new offsets and review the diff.
`multi-platform.yml:331-340` compiles the regenerated header with
`arm-linux-gnueabihf-g++ -std=c++17 -fsyntax-only`, because 32-bit ARM is where
alignment assumptions diverge first.

The Rust-side equivalent for message identity is
`horus_core/tests/message_layout_contract.rs`: `Topic::new` validates only the
type's short name and, for POD types, its size, so two revisions of a message
that keep the name and size while reordering fields both open the same topic and
the coordinates arrive swapped with no error (`:1-17`).
`Topic::new_checked(name, hash)`
(`horus_core/src/communication/topic/mod.rs:4244`) and the `LAYOUT_HASH` that
`message!` generates (`horus_core/src/communication/macros.rs:222`) are what
close it.
