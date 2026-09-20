# Contributing to HORUS

This document describes what actually gates a merge here, so you can get a
change in without discovering the rules one red run at a time.

Formatting is not optional and clippy warnings are not tolerated:
`cargo fmt --all -- --check` runs on every pull request without condition
(`.github/workflows/ci.yml:93`), and clippy runs with `-D warnings` whenever the
base branch is `main` (`:135-136`). Both feed the required `CI Success` check.

Everything below is transcribed from the workflow files and from the
branch-protection API, with a citation against each claim so you can check it
yourself. Where the two disagree, or where a rule is asserted nowhere but in a
maintainer's head, this document says so rather than inventing a tidy answer.

## Getting Started

1. **Fork the repository** on GitHub.
2. **Clone your fork**:
   ```bash
   git clone https://github.com/your-username/horus.git
   cd horus
   ```
3. **Branch from the branch you intend to target** (see the branch model below):
   ```bash
   git checkout main
   git pull origin main
   git checkout -b fix/your-change
   ```

### The branch model

`main` is the default branch (`gh api repos/softmata/horus --jq .default_branch`)
and the only protected one. `dev` exists and has no protection at all:
`gh api repos/softmata/horus/branches/dev/protection` returns 404, and the
GraphQL `branchProtectionRules` list contains exactly one node, whose pattern is
`main`.

Every workflow that produces a *required* check triggers on `pull_request`
against both `[main, dev]` — `ci.yml:8` (which also lists `master`),
`multi-platform.yml:8`, `integration-tests.yml:9`, `feature-matrix.yml:8`,
`cross-platform-parity.yml:16`, `docker-distro-tests.yml:13`,
`benchmarks.yml:7`. What differs for those seven is strictness, not coverage.

That is not true of everything. `safety.yml:18-19` restricts its
`pull_request` trigger to `branches: [main]`, so MIRI and the sanitizers never
run against a `dev` PR at all.

Four separate switches in CI key off `github.base_ref == 'main'`:

| Switch | Where | Targeting `dev` | Targeting `main` |
| --- | --- | --- | --- |
| `RUSTFLAGS` for every job in `ci.yml` | `ci.yml:28` | `-W warnings` | `-D warnings` |
| clippy | `ci.yml:134-140` | plain `cargo clippy` | `cargo clippy … -- -D warnings` |
| `cargo audit` | `ci.yml:302-306` | advisory, prints a warning | fails the job |
| `RUSTDOCFLAGS` for the doc job | `ci.yml:347` | empty | `-D warnings` |

The consequence is the thing to plan around. A branch can go entirely green
against `dev` while carrying a compiler warning, a clippy lint, an unreviewed
advisory and a broken rustdoc link. All four turn red on the `dev` → `main`
promotion, at which point they are someone else's problem in someone else's PR.
`ci.yml:20-27` records why the gate was moved onto `base_ref` in the first
place: keying on `github.ref` alone meant the strict path only ever ran *after*
a merge, because `github.ref` is `refs/pull/N/merge` for a `pull_request` event.
A PR could introduce a warning, pass, and break `main` where nothing could be
gated on it.

So: if your change is meant for `main`, target `main` and take the strict gates
now. Targeting `dev` is only a deferral.

Note also that pushing to `dev` runs no CI at all — `ci.yml:9` restricts the
`push` trigger to `[main, master]`. Work on `dev` is gated by pull requests or
not at all.

## What blocks a merge into `main`

Seven status checks are required. The requirement lives only in the
branch-protection API; no file in the repository records which checks are
required, and the two places that refer to the set at all are both wrong:
`autopilot.yml:5` says "the existing six required checks", and
`docs-contract.yml:503` calls its own aggregator "The single check branch
protection points at" when the API does not list it. This table is the
transcription:

| Required context | Produced by | Aggregates |
| --- | --- | --- |
| `CI Success` | `ci.yml:480` | `check, fmt, clippy, test, doc, security, loom, test-quality` (`:481`) |
| `Multi-Platform Success` | `multi-platform.yml:403` | `macos, macos-arm64, windows, linux-arm64, linux-armv7, msrv` (`:404`) |
| `Integration Tests Success` | `integration-tests.yml:1372` | `ipc-integration, cli-integration, generated-project, runtime-integration` (`:1374`) |
| `Feature Matrix Success` | `feature-matrix.yml:154` | `python, minimal, all-features` (`:155`) |
| `Parity CI Success` | `cross-platform-parity.yml:341` | `parity-linux, parity-macos, parity-windows, cross-compile, cli-parity` (`:342`) |
| `All Distros Pass` | `docker-distro-tests.yml:64` | one `docker-test` matrix job per distro: ubuntu, debian, fedora, alpine, arch (`:38`, `:65`) |
| `Run Benchmarks` | `benchmarks.yml:70` | not an aggregator; the benchmark job itself |

Alongside those, protection sets:

- **Strict mode is on** (`required_status_checks.strict: true`). Your branch must
  be up to date with `main` before the merge button unlocks, so every unrelated
  merge into `main` invalidates every queued branch and each one has to be
  updated and re-run. This is why `autopilot.yml` exists: `autopilot.yml:12-14`
  states the problem plainly — GitHub's auto-merge does not update branches, so
  queued PRs starve — and its convoy job takes the front of the queue
  (`autopilot.yml:238`) and calls `gh pr update-branch` on that one PR
  (`:242`), once per merge to `main` (`:23-24`). Expect your branch to be
  updated for you, eventually, and expect a full CI cycle each time it is. Keep
  PRs small; a large one spends most of its life re-running.
- **Conversation resolution is required** (`required_conversation_resolution:
  enabled`). Every review thread on the PR must be marked resolved before it can
  merge. One active repository ruleset (`gh api repos/softmata/horus/rulesets`,
  id 19245331) applies `copilot_code_review` to the default branch with
  `review_on_push: true` and `review_draft_pull_requests: true`, so a bot
  re-reviews on every push to a PR targeting `main`, drafts included. Threads
  can therefore appear without a human opening them. Whether GitHub's resolution
  requirement counts those particular threads is standard platform behaviour
  that has not been tested here — treat it as unverified, and resolve them
  anyway.
- **Linear history is required** and force pushes to `main` are refused
  (`required_linear_history.enabled: true`, `allow_force_pushes.enabled:
  false`).
- **No approving review is required by protection.** GraphQL
  `branchProtectionRules` reports `requiresApprovingReviews: false` and
  `requiredApprovingReviewCount: null`, and the top-level REST protection object
  omits `required_pull_request_reviews` entirely. The REST sub-endpoint
  `…/protection/required_pull_request_reviews` contradicts both, returning
  `{"required_approving_review_count": 1, …}`. The two disagree; this document
  follows GraphQL and the top-level object, and flags the contradiction rather
  than picking silently. Treat review as a project norm and confirm the
  mechanical state before relying on either reading.
- **Admins are not bound** (`enforce_admins.enabled: false`).

### Checks that run on your PR and do not block it

These are worth reading when they go red, because nothing stops a merge if they
do:

- `Docs Contract Success` (`docs-contract.yml:508`), which aggregates the three
  jobs that run on a PR: `CLI contract (hermetic)` (`:110`), `CLI contract (live
  docs)` (`:173`) and `Shipped examples build` (`:290`) — the needs list is at
  `:510`. The workflow's other two jobs, `Rust examples compile` (`:337`) and
  `Python API references exist` (`:440`), are gated to `schedule` or an explicit
  `run_compile_sweep` dispatch (`:338`, `:441`), so a PR never runs them, and
  they are not in the aggregator's needs list either.
- `MIRI - Undefined Behavior Detection` (`safety.yml:42`), `AddressSanitizer -
  Memory Error Detection` (`:102`) and `ThreadSanitizer - Data Race Detection`
  (`:165`). These trigger only on PRs targeting `main` and only when the diff
  touches `horus_core/src/memory/**` or `horus_core/src/communication/**`
  (`safety.yml:18-23`). A PR into `dev` that changes unsafe memory code runs
  none of them.
- The C++ binding suites (`cpp-bindings.yml`, path-filtered to
  `horus_cpp/**`, `horus_core/**`, `horus_types/**`, `horus_library/**`),
  coverage (`coverage.yml`, unfiltered), monitor tests (`monitor-tests.yml`,
  path-filtered), container images (`container-images.yml`, path-filtered to
  `Dockerfile`, `.dockerignore`, `.devcontainer/**`, its own file, `**/Cargo.toml`
  and `Cargo.lock`) and `distribution.yml` (unfiltered).

## Development Setup

### Prerequisites

- **Rust.** Most CI jobs use `stable`. The workspace declares
  `rust-version = "1.90"` (`Cargo.toml:28`), but the MSRV job pins and tests
  `1.92.0` (`multi-platform.yml:26`, `:350`). The floor CI enforces on Rust
  source is 1.92.0; the manifest has not caught up. 1.90 is not entirely
  untested — the root `Dockerfile:65` is `FROM rust:1.90-slim-bookworm` and
  `container-images.yml` builds it — but that workflow is path-filtered and
  produces no required context, so a source change that breaks 1.90 can merge.
  There is no `rust-toolchain.toml` (`ls rust-toolchain*` finds nothing), so your
  default toolchain is what gets used.

  One trap in that job: `MSRV: "1.92.0"` at `multi-platform.yml:26` is never
  referenced by any step. The version that runs comes from the matrix at `:350`.
  Bumping the env var alone changes nothing.
- **System packages.** The test job installs
  `libudev-dev pkg-config cmake libeigen3-dev libfmt-dev libgtest-dev`
  (`ci.yml:156-157`); the last four are needed because `cmake_gen`'s
  `real_cmake_*` tests shell out to `cmake` and `find_package(Eigen3/fmt/GTest)`
  (`ci.yml:154-155`). Jobs that only build Rust install `libudev-dev pkg-config`
  alone (`ci.yml:49`, `:105`, `:318`, `:359`).
- **Python 3** with `venv`, if you touch `horus_py` or any cross-language test.
- **`rustfmt` and `clippy` components** (`rustup component add rustfmt clippy`).

There is a devcontainer if you would rather not install any of it:
`.devcontainer/devcontainer.json` builds the repository `Dockerfile`'s
`devcontainer` stage (`:14-17`) and runs with `--shm-size=1g --ipc=host`
(`:30`). Both flags matter, and the file says why (`:27-29`): the default
64 MB `/dev/shm` is too small for image or point-cloud topics, and `--ipc=host`
is what lets nodes in the container reach nodes on the host.

### Environment a test run needs

Two variables. Neither is named in any markdown file in this repository — a
`grep -rl` for them over `--include=*.md` returns nothing. The docs site
describes both for *users*
(`content/docs/development/environment-variables.mdx:79` and `:132` in
`softmata/horus-docs`); what follows is what a *test run* needs.

**`HORUS_SOURCE`.** `horus_manager`'s `cargo_gen` / `cmake_gen` / `new` / `pkg`
tests generate `.horus` build files against a real HORUS source tree.
`find_horus_source_dir()` (`horus_manager/src/commands/run/run_rust.rs:1046-1116`)
resolves it in three stages: `HORUS_SOURCE` first (`:1050-1056`), then a fixed
candidate list — `/horus`, `~/softmata/horus`, `~/horus`, `/opt/horus`,
`/usr/local/horus` (`:1059-1065`) — and finally the installer's own source cache
under `<cache>/horus@<version>` (`:1074-1106`). Two workflow comments
(`ci.yml:29-32`, `integration-tests.yml:30-33`) describe only the middle stage
and omit the cache; `docs-contract.yml:296-297` gets it right.

Every candidate, including the one you pass in `HORUS_SOURCE`, is accepted only
if it contains `horus/Cargo.toml` (`:1052`, `:1068`, `:1084`, `:1100`). A path
that fails that check is silently ignored and the search continues, so set it
from the repository root:

```bash
export HORUS_SOURCE="$PWD"     # must contain horus/Cargo.toml
```

CI does the same in seven workflows: `ci.yml:33`, `integration-tests.yml:34`,
`multi-platform.yml:31`, `safety.yml:35`, `coverage.yml:24`,
`monitor-tests.yml:44` and `docs-contract.yml:301`. `distribution.yml:51`
deliberately does not set it.

**`HORUS_NAMESPACE`.** Shared memory lives under `/dev/shm/horus_<namespace>` on
Linux, `/tmp/horus_<namespace>` on macOS and `%TEMP%\horus_<namespace>` on
Windows (`horus_sys/src/shm/mod.rs:242-262`). `generate_namespace()`
(`:186-207`) takes `HORUS_NAMESPACE` first, sanitised to `[A-Za-z0-9_]` with
everything else replaced by `_` (`:37-47`); failing that, a test binary derives a
namespace private to the directory cargo built it into (`:70-89`); and failing
*that*, a process lands in the shared `"default"` namespace, which is the ROS 2
domain-0 model — every process on the machine sees every other.

So a plain `cargo test` is already isolated from a running robot, but `horus
run` is not. Two concurrent test runs sharing one target directory derive the
same namespace and will collide; give them distinct `HORUS_NAMESPACE` values.
CI sets it per run for that reason (`ci.yml:147`, `integration-tests.yml:42`).

The full local equivalent of a CI test environment:

```bash
export HORUS_SOURCE="$PWD"
export HORUS_NAMESPACE="local_$$"
export RUST_BACKTRACE=1          # ci.yml sets this on every test step

# The five subdirectories CI creates, owner-only. Do not chmod 777:
# regression_shm_topics_dir_permissions_restricted (horus_core/tests/regressions.rs:307)
# asserts the topics dir is exactly 0o700, and a permissive mkdir breaks it.
SHM_DIR="/dev/shm/horus_${HORUS_NAMESPACE}"
mkdir -p "$SHM_DIR"/{topics,nodes,control,network,scheduler}
chmod -R 700 "$SHM_DIR"
```

That mirrors `ci.yml:179-186`. The same block appears twelve times across six
workflow files — `ci.yml:185`, `integration-tests.yml:76`, `:348`, `:740`,
`:1014`, `:1170`, `safety.yml:70`, `:138`, `:190`, `multi-platform.yml:376`,
`coverage.yml:84` and `benchmarks.yml:159`. `monitor-tests.yml` uses a variant
with an extra `topics/horus_topic` level (`:91`, `:145`, `:187`). Do not create
these with `sudo` — root-owned directories are what made
`SchedulerRegistry::open` (`horus_core/src/scheduling/registry.rs:158`) and the
presence writer fail with `PermissionDenied` (`ci.yml:180-184`).

The IPC integration job additionally raises the kernel limits
(`integration-tests.yml:79-80`):

```bash
sudo sysctl -w kernel.shmmax=2147483648
sudo sysctl -w kernel.shmall=2147483648
```

Leaked namespaces are a real and recurring cause of unrelated test failures.
`autopilot.yml:82-83` tells its own triager to clear them first, because leaked
namespaces make unrelated tests fail in a rotating pattern. If a suite starts
failing in a pattern that does not match your change:

```bash
rm -rf /dev/shm/horus_*
```

For real-time behaviour on Linux, `scripts/setup-realtime.sh` sets the CPU
governor to `performance`, configures `SCHED_FIFO`/`SCHED_RR` limits up to
priority 99, raises the memlock limit for `mlockall`, installs a udev rule for
shared-memory permissions, and optionally installs `cpupower`/`cpufrequtils`
(`scripts/setup-realtime.sh:5-10`). `sudo ./scripts/setup-realtime.sh`
configures the machine; `sudo ./scripts/setup-realtime.sh --check` reports the
current state without changing anything (`:12-14`). You must log out and back in
for the limit changes to take effect (`:23`). The script refuses to run on
anything but Linux (`:28-32`) and requires root (`:35-39`).

### Building

```bash
cargo build --workspace --exclude horus_py

# Python bindings, into a venv:
python3 -m venv .venv
.venv/bin/pip install --upgrade pip maturin
.venv/bin/maturin develop --release --manifest-path horus_py/Cargo.toml
```

`horus_py` is excluded from every workspace-wide Rust build and test in this
repository, and the reason is worth knowing before you hit it.
`horus_py/src/lib.rs:11-18` raises a `compile_error!` whenever `cfg(test)` and
the default `extension-module` feature are both on: pyo3 leaves Python's symbols
to the interpreter that `dlopen`s the `.so`, so a test binary has nothing to link
them against. Without the guard you get roughly forty lines of `rust-lld: error:
undefined symbol: PyEval_RestoreThread` (`horus_py/src/lib.rs:7-9`). Cargo
cannot vary features per target, so the crate gets a second pass with the
feature off instead (`ci.yml:66-78`, `:224-227`).

## Testing

### Run the timing-sensitive tests in `--release`

CI does — its whole `tests/` sweep is `--release` (`integration-tests.yml:127`)
— and it matters: a debug build is roughly an order of magnitude slower, so a
1 kHz RT scheduler test, a latency percentile check or a stress test can fail in
`debug` purely because the code cannot keep up. Nothing is wrong with the code.
`autopilot.yml:79-80` gives the same instruction to its own triager: timing,
throughput and tick-rate assertions are only meaningful with `--release`.

The suites where this bites are `horus_core/tests/stress_rt_contention.rs`,
`scheduler_config_test.rs`, `production_fanout_battle.rs` and
`safety_battle_tests.rs`. All four exist. The current `CONTRIBUTING.md:49-52`
puts a number on it — "Seven tests" across those four files — which is not
verifiable without running both configurations, so it is not repeated here. If a
test in one of those fails, re-run it in `--release` before you start reading the
diff.

### What CI runs

Reproduce a failure in the configuration it happened in.

```bash
# The `test` job in ci.yml (:203-208) — unit tests only, serialized,
# with four cross-thread/robotics tests skipped.
cargo test --workspace --exclude horus_py --lib --no-fail-fast -- \
  --test-threads=1 \
  --skip topic_cross_thread_1p_multi_c_spmc \
  --skip topic_cross_thread_mpmc_pre_initialized_99_percent \
  --skip topic_cross_thread_multi_p_multi_c_mpmc \
  --skip test_robotics_autonomous_car_perception

# The `ipc-integration` job (integration-tests.yml:127-133) — every tests/
# target, in RELEASE, serialized, with five timing-sensitive tests skipped.
# That job also sets RUST_LOG=debug and PYTHONPATH=<repo>/horus_py
# (integration-tests.yml:137-138).
cargo test --workspace --exclude horus_py --release --test '*' -- \
  --test-threads=1 --nocapture \
  --skip shm_fanout_latency_percentiles_cross_thread \
  --skip stress_rt_isolation_from_compute_nodes \
  --skip stress_rt_1khz_under_cpu_contention \
  --skip stress_rt_1khz_baseline_no_contention \
  --skip stress_rt_multi_rate_under_contention

# The `cli-integration` job (integration-tests.yml:794) — the whole
# horus_manager package, integration targets included.
cargo test -p horus_manager --release -- --test-threads=1

# horus_py, Rust side (ci.yml:227) and Python side (ci.yml:256-257).
cargo test -p horus_py --no-default-features -- --test-threads=1
cd horus_py && python -m pytest tests/ -p no:cacheprovider
```

Add `--no-fail-fast` when you want the whole failure list: without it cargo stops
at the first failing test binary, so a run reports one failure at a time and you
fix them one slow round-trip apiece (`autopilot.yml:84-85`).

### The `#[ignore]`d suites, which `--test '*'` does not reach

`--test '*'` builds every integration target but runs none of its `#[ignore]`d
tests, and for a long time nothing passed `--ignored` for `horus_core`
(`integration-tests.yml:140-146`). Several suites use `#[ignore]` as a mechanism
rather than as neglect: they re-exec their own test binary as child processes so
publisher and subscriber are genuinely separate processes (`:153-158`). Three
extra steps now run them, and they are not equal:

- **Gating** (`integration-tests.yml:219-231`): `ipc_torture`, `chaos_monkey`,
  `message_type_matrix`, `hardware_emulation`, `params_intent`,
  `log_system_tests`, `rust_python_full_matrix`, `rust_python_matrix`,
  `humanoid_stress`, `matrix_sweep`. A failure here fails a required context.
- **Advisory, `continue-on-error`** (`:244-264`): `chaos_cross_process`,
  `production_validation`, `chaos_xp_schedulers`, `production_fanout_battle`.
  These spawn five to ten real child processes on a two-vCPU runner, and the file
  is explicit that a failure there does not distinguish a HORUS regression from a
  busy neighbour (`:236-243`).
- **Advisory for a different reason** (`:289-297`): `multi_scheduler_matrix`,
  kept separate because it exposes a real fault rather than an environment
  limit — `deterministic_alongside_normal_scheduler` dies with SIGBUS, reproduced
  two runs in five on a twelve-core box (`:269-288`). It goes back to gating when
  the topic-layer fix lands.

Three further suites (`cli_runtime_integration`, `launch_and_workflow`,
`real_project_ipc`) resolve `target/debug/examples/...` by hardcoded path and are
left out entirely until that prerequisite is wired (`:160-166`, `:181-187`).

To run an ignored suite yourself:

```bash
cargo test -p horus_core --release --test ipc_torture -- --ignored --test-threads=1 --nocapture
```

### The five tests that run nowhere

**The five tests skipped in the release sweep run nowhere in CI.**
`integration-tests.yml:114-123` says so directly, and the negative holds: `grep
-rn stress_rt .github/workflows/` matches only the two skip lists
(`integration-tests.yml:130-133` and `coverage.yml:118`), and
`shm_fanout_latency_percentiles_cross_thread` appears only at
`integration-tests.yml:129` and `coverage.yml:118`. It is not `#[ignore]`d
(`horus_core/tests/production_fanout_battle.rs:157-158`), so the `--ignored`
advisory step does not pick it up either.

They are developer-run gates: on a shared two-vCPU runner they measure the
runner rather than HORUS — that job saw 34.24% spread
(`integration-tests.yml:108-112`). If you change the tick path, run them
yourself on a machine you control. The workflow names the invocation
(`:123`):

```bash
cargo test -p horus_core --release --test stress_rt_contention -- --nocapture
```

### `horus_py` coverage

`horus_py` is excluded from every workspace-wide sweep, but it is not
untested. Two things run:

- `cargo test -p horus_py --no-default-features -- --test-threads=1`
  (`ci.yml:227`), which covers the 67 `#[test]` functions in `horus_py/src`.
- The pytest suite (`ci.yml:240-257`), which is the only `pytest` invocation in
  any workflow — a grep for `pytest` over `.github/workflows/` matches only
  `ci.yml:235`, `:243` and `:257`.

The comment at `ci.yml:231-234` says the Python suite is "the ONLY coverage the
horus_py binding layer has". Read that as being about the built extension, not
about the crate: `:227` is Rust coverage of the same crate, and it runs in the
same job.

The Python suite has a trap. `horus_py/horus/__init__.py:227-236` catches
`ImportError` and falls back to a "mock mode" with `Topic = None` and a
`RuntimeWarning`, so `import horus` succeeds even when the extension was never
built — and the whole suite then runs green against a null API. CI asserts
against this before running pytest (`ci.yml:244-255`). Do the same locally:

```python
import horus, sys
if horus.Topic is None:
    sys.exit("horus imported in MOCK MODE — the extension did not build")
```

### Loom models

`horus_core/tests/` holds ten `loom_*` targets. The dedicated `loom` job runs six
of them by name (`ci.yml:382-421`): `loom_fanout`, `loom_pod_broadcast`,
`loom_concurrency`, `loom_migration`, `loom_migration_data_plane`,
`loom_participant`. The other four — `loom_mp_claim`, `loom_sp_mp_flag`,
`loom_spmc_epoch_flush`, `loom_tensor_pool` — are not named there. They carry no
`cfg(loom)` gate (a grep for `cfg(loom)` over `horus_core/tests/` returns
nothing) and `loom` is a plain dev-dependency (`horus_core/Cargo.toml:69`), so
they are ordinary integration targets matched by `integration-tests.yml:127`'s
`--test '*'`.

`loom_pod_broadcast` carries a `naive` variant that reproduces the old protocol
and is *supposed* to fail (`ci.yml:387-390`) — that failure is what proves the
model is not vacuous. Do not "fix" it.

### The C++ layout contract

`horus_cpp/include/horus/layout_contract.hpp` is rendered from the Rust types
with `offset_of!` and committed (`horus_cpp/src/layout_contract.rs:30-33`).
`contract_file_is_current` (`:213-214`) compares the rendering against the
committed header and fails if they have drifted, so a message type that gains,
loses, renames or reorders a field turns red. The test module is
`#[cfg(test)]` inside the lib (`:189`, declared at `horus_cpp/src/lib.rs:33`), so
it runs in the workspace `--lib` sweep and sits inside `CI Success`.

When the failure is intended, regenerate deliberately and review the diff:

```bash
cargo test -p horus_cpp --no-default-features regenerate_layout_contract -- --ignored --nocapture
```

That is the canonical form, quoted verbatim from `layout_contract.rs:38`. It
also appears at `:169` and inside the failure message at `:221`, and nowhere else
in the tree — no workflow and no markdown file mentions it. Note that
`horus_cpp/Cargo.toml` declares no `[features]` section, so
`--no-default-features` does nothing for that crate; it is harmless, and it is
what the file tells you to type.

### Feature combinations

`--workspace --no-default-features` does *not* produce a minimal build: Cargo
unifies features across the workspace, so siblings that depend on `horus_core`
or `horus` with default features re-enable them anyway
(`feature-matrix.yml:105-110`). The only way to catch a no-default-features
regression is per crate, which is what CI does (`:111-115`):

```bash
cargo check -p horus_core --no-default-features
cargo check -p horus --no-default-features
cargo check -p horus_types --no-default-features
cargo check -p horus_net --no-default-features
cargo check -p horus_sys --no-default-features
cargo check --workspace --all-features       # feature-matrix.yml:150
```

If you add a workspace member, add it to that list, or it has no
no-default-features floor.

### Acceptance tests

- **Rust** — `horus_core/tests/acceptance_topic.rs` and
  `horus_core/tests/acceptance_scheduler.rs`.
- **Cross-platform CLI** — `tests/docker/acceptance_linux_macos.sh` and
  `tests/docker/acceptance_windows.ps1`.
- **End-to-end QA scenarios** — `tests/qa/launch`, `tests/qa/pubsub_rust`,
  `tests/qa/pubsub_python`.

```bash
cargo test -p horus_core --test acceptance_topic
cargo test -p horus_core --test acceptance_scheduler
bash tests/docker/acceptance_linux_macos.sh

horus new test_project && cd test_project && horus run
```

### Benchmarks

`Run Benchmarks` is a required check. It runs two binaries —
`cross_process_benchmark` and `robotics_messages_benchmark` — `BENCH_REPETITIONS`
times each, which is 3 (`benchmarks.yml:53`, `:179-188`), and feeds the JSON
reports to a `regression_gate` binary
(`benchmarks/src/bin/regression_gate.rs`, built at `benchmarks.yml:162-166`,
invoked at `:242-249`). The gate reduces the repetitions to their median so one
bad draw cannot fail the build (`:48-52`) and compares against a rolling window
of `BENCH_WINDOW` past trunk runs, which is 20 (`:55-58`).

There is no single threshold: each metric is gated at `max(relative, center +
k·σ_observed, absolute floor)`, and the policy, the blocking/report-only split
and the reasoning for every number live in `RegressionPolicy` in
`benchmarks/src/output.rs:369` (`benchmarks.yml:38-45`). The figure of merit is
worst-case latency and jitter first, median second — a change that improves the
median and widens the tail is a regression here (`:25-27`).

Two things only happen on a push to `main` (`:237-240`): `--record`, which
rewrites the rolling baseline, and `--trunk`, which enables the
consecutive-runs escalation. Both are deliberately off for pull requests, so
your PR is compared against trunk but never becomes its own baseline.

```bash
cargo run --release --bin cross_process_benchmark
cargo run --release --bin robotics_messages_benchmark
```

The five `[[bench]]` targets in `benchmarks/Cargo.toml:84-102` — `topic_latency`,
`topic_throughput`, `scheduler_jitter`, `scheduler_ipc_latency`, `tensor_pool` —
are a separate thing and the gate does not read them.

### Documentation contract

`horus-docs` is a separate repository, and this repository's tests can break its
published pages. On a PR, `docs-contract.yml` runs a hermetic tier needing no
docs checkout (`:109-167`), a live tier that clones `softmata/horus-docs` at a
pinned commit (`:172-183`), and a shipped-examples build (`:289-334`).

The pin is `DOCS_PINNED_REF` (`docs-contract.yml:103`). If your change requires a
docs change to stay true, land the docs change first, then bump the pin in a PR
here. Verify the new commit before you bump it — a pin is a claim about a
specific tree (`:58-65`):

```bash
HORUS_DOCS_DIR=<horus-docs checkout> \
  cargo test -p horus_manager --test docs_contract -- --include-ignored
HORUS_DOCS_DIR=<horus-docs checkout> \
  cargo test -p horus_manager --test docs_cli_contract
```

`HORUS_DOCS_DIR` must point at a directory containing `content/docs`, or it is
ignored with a warning (`horus_manager/tests/docs_contract.rs:61-68`). Without a
checkout these tests *skip* locally and *fail* under `CI`
(`horus_manager/tests/docs_contract.rs:19-21`, `:100`), so a green local run
proves nothing if you forgot the variable.

There is deliberately no workflow input to override the pin. `schedule` and
`workflow_dispatch` run in default-branch context with write access to the
Actions cache, so building an arbitrary ref of another repository here would let
anyone who can push to `softmata/horus-docs` execute code in this repository's
privileged context (`docs-contract.yml:86-93`).

## Code Style

### Rust

- **Formatting is mandatory.** `ci.yml:93` runs
  `cargo fmt --all -- --check` with no condition and no `continue-on-error`; the
  `fmt` job is in `ci-success`'s `needs` list (`ci.yml:481`) and its result is
  checked (`ci.yml:490`). A formatting diff fails a required check on every PR,
  whichever branch you target.
- **Clippy warnings fail when the PR targets `main`.** `ci.yml:135-136` runs
  `cargo clippy --workspace --all-targets --exclude horus_py -- -D warnings` plus
  a second `-p horus_py --all-targets --no-default-features` pass. Targeting
  `dev` drops the `-D warnings` (`ci.yml:138-139`), which defers the problem
  rather than removing it.
- **Compiler warnings fail when the PR targets `main`.** `RUSTFLAGS` is set at
  the workflow level (`ci.yml:28`), so `-D warnings` applies to `check`, `test`,
  `doc` and `loom` as well as clippy.
- **Rustdoc warnings fail when the PR targets `main`** (`ci.yml:347`). Note that
  `cargo doc` excludes `horus_manager` (`ci.yml:345`) to avoid a
  `target/doc/horus/` filename collision with the `horus` binary
  (`ci.yml:337-338`), so doc warnings in that crate are not caught. `horus_py`
  needs no split here — rustdoc never sets `cfg(test)`, so the
  `extension-module` guard does not fire (`ci.yml:340-344`).

Run the strict configuration locally before you push:

```bash
cargo fmt --all -- --check
RUSTFLAGS="-D warnings" cargo clippy --workspace --all-targets --exclude horus_py -- -D warnings
RUSTFLAGS="-D warnings" cargo clippy -p horus_py --all-targets --no-default-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps --exclude horus_manager
cargo audit          # cargo install cargo-audit --locked, if you have not
```

Document public APIs with `///`:

```rust
/// Creates a new Topic for inter-process communication.
///
/// # Arguments
///
/// * `topic_name` - The topic name for this channel
///
/// # Examples
///
/// ```
/// let topic = Topic::<f32>::new("temperature")?;
/// ```
pub fn new(topic_name: &str) -> Result<Self> {
    // implementation
}
```

### Unsafe code

`horus_core/src` holds 340 lines containing `unsafe {` and 253 lines carrying a
`// SAFETY:` comment. **Nothing enforces the correspondence.** There is no
`undocumented_unsafe_blocks`, `missing_safety_doc`, `unsafe_op_in_unsafe_fn` or
`deny(unsafe…)` in the workspace lints (`Cargo.toml:83-102`) or anywhere else —
a grep for those names across every `.rs` and `.toml` in the repository returns
nothing. The only mechanisms are the PR template's safety checklist
(`.github/PULL_REQUEST_TEMPLATE.md:35-40`) and review.

So write the comment because the next person needs it, and be aware of what the
machine will and will not do for you:

- MIRI, ASan and TSan run only on PRs targeting `main` that touch
  `horus_core/src/memory/**` or `horus_core/src/communication/**`
  (`safety.yml:18-23`), and none of the three is a required check.
- MIRI cannot execute most of `horus_core` at all: it does not support
  file-backed memory mappings, which since the transport went SHM-only rules out
  every `Topic`, the `TensorPool` and every pool-backed type
  (`safety.yml:75-82`). The file records that each of the three steps that used
  to live there died on the first such test, so the job has never reported a UB
  result. What it runs now is the SIMD copy helpers, the RT allocator guard and
  the SHM path helpers (`:84-86`):

  ```bash
  MIRIFLAGS="-Zmiri-disable-isolation -Zmiri-permissive-provenance" \
    cargo miri test -p horus_core --lib -- memory::simd memory::rt_allocator memory::platform
  ```

  (`safety.yml:92-95`; the three modules are `horus_core/src/memory/simd.rs`,
  `rt_allocator.rs` and `platform.rs`.) The mmap-backed paths are covered
  instead by ASan, TSan and the `Valgrind Memcheck` job in
  `cpp-bindings.yml:540-541` (`safety.yml:88-89`).

### Test quality

`ci.yml:434-476` greps the tree for stub-test anti-patterns and fails the build
on two of them:

| Pattern | Why it fails |
| --- | --- |
| `is_ok() \|\| is_err()` (either order), `ci.yml:440` | always true; asserts nothing |
| a bare `matches!(…);` statement not preceded by an `assert`, `ci.yml:448-463` | the `bool` is discarded |

A third, `let _ = format!(…)` in test code, emits a warning rather than failing
(`ci.yml:467-469`).

One local caveat: the guard runs `grep -rn … --include='*.rs' .` from the
repository root with no `--exclude-dir` (`ci.yml:440`, `:448`, `:467`). A CI
checkout has no `target/`; yours does, so reproducing the scan locally will also
walk generated sources under it. Point the grep at the crate directories rather
than at `.`.

### Python

Follow PEP 8 and use descriptive names — but be aware that **no check enforces
any of it**. A case-insensitive grep over `.github/workflows/` finds no `black`,
`mypy`, `ruff` or `flake8` invocation anywhere. The only automated Python gate is
the pytest suite in `ci.yml:240-257`.

## What to Contribute

### Good first issues

Issues labelled `good first issue` — the label name has spaces, and the
repository's label set also carries `help wanted`, `bug`, `enhancement`,
`documentation`, `testing`, `rust` and `CLI tools`.

### Feature requests

Before implementing a major feature, open an issue, wait for maintainer
feedback, then implement with tests and documentation. The seven required checks
take a while; discovering after the fact that a design is unwanted is expensive
for everyone.

### Bug reports

Include the HORUS version (`horus --version`, wired at
`horus_manager/src/main.rs:14`), the operating system and version, a minimal
reproducible example, expected versus actual behaviour, and the relevant logs.
`horus doctor` (`horus_manager/src/main.rs:852`, dispatched at `:3557`) checks
the machine — toolchains, real-time capability, shared memory (`:848`) — and
takes `--verbose`, `--json`, `--fix` and `--rt`. `horus check` checks the
repository; `:850` notes that neither substitutes for the other.

## Pull Request Process

1. **Choose the base branch deliberately.** `main` takes the strict gates now;
   `dev` defers them.

2. **Run the gates locally.** At minimum:
   ```bash
   cargo fmt --all -- --check
   RUSTFLAGS="-D warnings" cargo clippy --workspace --all-targets --exclude horus_py -- -D warnings
   cargo test --workspace --exclude horus_py --lib --no-fail-fast -- --test-threads=1
   ```

3. **Update the acceptance tests** in `horus_core/tests/acceptance_*.rs` and
   `tests/docker/` if behaviour changed, and add scenarios for new features.

4. **Update documentation.** README.md for user-facing changes, inline `///`
   docs, examples, and `CHANGELOG.md`. The changelog is machine-checked:
   `horus_manager/tests/changelog_contract.rs` requires that a changelog exists
   (`:51`), that every released git tag appears in it (`:63`), that the version
   in `env!("CARGO_PKG_VERSION")` — `horus_manager`'s own package version, 0.4.0
   at `horus_manager/Cargo.toml:3`, since there is no `[workspace.package]
   version` to inherit — has either its own `[x.y.z]` section or an
   `## Unreleased` section to collect into (`:88-96`), and that releases are
   listed newest first (`:101`).

   Those nine tests, plus `translation_contract.rs`'s five and ten of
   `readme_contract.rs`'s eleven, run inside the required
   `Integration Tests Success` context. No workflow names any of the three
   files; they run because `integration-tests.yml:794` is
   `cargo test -p horus_manager --release -- --test-threads=1` with no target
   filter, `horus_manager/Cargo.toml` declares no `[[test]]` blocks and no
   `required-features`, so every file in `horus_manager/tests/` is autodiscovered,
   and line 794 sits inside the `cli-integration` job (`:309`) which is in the
   aggregator's needs list (`:1374`).

   `readme_contract.rs`'s `the_readme_quick_start_compiles` is `#[ignore]`d
   with the reason "run in the docs-contract job" (`readme_contract.rs:146`),
   and that job does now run it — `docs-contract.yml` invokes
   `cargo test -p horus_manager --test readme_contract -- --include-ignored`.

   This paragraph previously said the opposite ("Nothing runs it: no workflow
   mentions `readme_contract`") and told you to compile the Quick Start by hand.
   That was true when it was written and stopped being true when the invocation
   was added; the prose was not updated with it. You can still run it locally,
   but CI will catch a broken Quick Start without you:

   ```bash
   cargo test -p horus_manager --test readme_contract -- --include-ignored
   ```

5. **Write clear commit messages.** `git-cliff` groups the changelog by
   conventional-commit prefixes — `feat`, `fix`, `doc`, `perf`, `refactor`,
   `style`, `test`, `chore`, `ci`, `deps` (`.github/cliff.toml:47-61`) — so a
   prefixed subject lands in the right section. `chore(release)` is skipped
   entirely (`:55`), `chore(deps)` and `deps` both land under Dependencies
   (`:56`, `:59`), and a commit whose *body* matches `security` is grouped under
   Security regardless of prefix (`:60`).

   ```
   fix: brief description

   Detailed explanation of what changed and why.

   - Updated acceptance tests in horus_core/tests/acceptance_*.rs

   Fixes #123
   ```

6. **Fill in the PR template** (`.github/PULL_REQUEST_TEMPLATE.md`). It appears
   automatically. Its safety (`:35-40`) and performance (`:42-46`) sections are
   the ones reviewers read most closely.

7. **Sign the CLA on your first PR.** `.github/CLA.md:71` states that making a
   contribution already binds you to the agreement; the requested signal is a
   comment on the pull request:

   ```
   I have read and agree to the Contributor License Agreement.
   ```

   **No bot checks for it** — a case-insensitive grep for `cla` and
   `contributor license` over `.github/workflows/` matches nothing — so nothing
   will remind you and nothing will block the merge. A maintainer will ask if it
   is missing. For significant contributions you may be asked to sign a physical
   copy (`.github/CLA.md:77`).

8. **Resolve every review thread.** Protection requires it. The Copilot reviewer
   re-runs on every push to a PR targeting `main`, so expect threads you did not
   open.

9. **Expect to be rebased.** Strict mode plus a busy `main` means your branch
   will be updated repeatedly and re-run each time.

## Architecture Guidelines

### Core principles

1. **Zero-copy when possible** — use shared memory.
2. **Type safety** — leverage Rust's type system.
3. **Minimal latency** — profile and optimize hot paths; judge on the tail, not
   the median (`benchmarks.yml:25-27`).
4. **Multi-language** — features should work across Rust, C++ and Python.

### Code organization

```
horus/
├── horus/              # Unified entry crate (horus::prelude, horus/src/lib.rs:581)
├── horus_core/         # Core framework (core, communication, memory, scheduling, types, services, actions, drivers)
├── horus_types/        # Universal IPC types (math, diagnostics, time, generic)
├── horus_macros/       # Procedural macros (node!, LogSummary derive)
├── horus_sys/          # Platform abstraction layer
├── horus_net/          # Transparent LAN replication
├── horus_cpp/          # C++ FFI bridge (CXX; crate-type staticlib, cdylib, rlib)
├── horus_cpp_macros/   # C++ binding codegen (#[horus_api] proc macro)
├── horus_manager/      # CLI tool (horus command)
├── horus_py/           # Python bindings
├── benchmarks/         # Performance benchmarks (package name: horus_benchmarks)
├── examples/           # Example projects (Rust, C++, Python)
└── tests/              # Integration and QA tests
```

The eleven crates are the workspace members listed in `Cargo.toml:3-15`, with
the per-crate descriptions taken from the comments there and from each crate's
own manifest. `examples/` and `tests/` are not workspace members. `node!` is the
proc macro at `horus_macros/src/lib.rs:76-79`; `LogSummary` is the derive at
`:102`.

Standard message types come from `horus_types/` and the external `horus-robotics`
git dependency, pinned by rev in `[workspace.dependencies]` (`Cargo.toml:40`).
`TransformFrame` comes from the external `horus-tf` git dependency, which is
declared per crate rather than in the workspace table — `horus/Cargo.toml:25`,
`horus_core/Cargo.toml:73`, `horus_cpp/Cargo.toml:21`,
`horus_manager/Cargo.toml:38`, `horus_py/Cargo.toml:31` — and re-exported through
`horus::prelude` at `horus/src/lib.rs:613`. Both git deps are patched to this
workspace's local `horus_core`, `horus_types` and `horus_macros`
(`Cargo.toml:108-115`), because they reference `horus_core` by relative path
(`Cargo.toml:105-107`).

## What Not to Do

- Break existing APIs without a migration path.
- Add dependencies without discussion. `cargo audit` is a gate on `main`
  (`ci.yml:302-303`), and `.cargo/audit.toml` is where a tolerated advisory has
  to justify itself. That list is currently **empty** (`ignore = []`), which the
  file says is the target state: an ignore entry suppresses its advisory for
  every matching version forever, including one that lands in the lockfile years
  later. Removed entries are kept above it as commented history with the reason
  each was retired. If you need to add one, name why it is tolerated and what
  would let us remove it.
- Widen the `/dev/shm` permissions to make a test pass.
  `regression_shm_topics_dir_permissions_restricted`
  (`horus_core/tests/regressions.rs:307-323`) asserts the topics directory is
  exactly `0o700`, and that assertion is the guarantee, not the obstacle.
- "Fix" `loom_pod_broadcast`'s `naive` variant. It is supposed to fail.
- Commit without running the tests, or open a PR with no description.
- Ignore a red check on the grounds that it is not required. Most red CI here is
  the runner rather than the code (`autopilot.yml:11`), but that is a conclusion
  to reach after reading the log, not before. `autopilot.yml:88-89` adds the
  corollary: detection of the concurrency bugs here is probabilistic, so passing
  once is not evidence of a fix, and `:93-95` asks for an ablation — show the
  test fails without your change and passes with it.

## Code Review

All contributions go through review. Be respectful and constructive, respond to
feedback promptly, ask when something is unclear, and expect maintainers to have
the final say. Whether protection mechanically requires an approval is
genuinely unclear — the two GitHub APIs disagree, as recorded above — so treat
review as a norm the project keeps rather than something to rely on the machine
to enforce.

## Code of Conduct

`CODE_OF_CONDUCT.md` applies to all project spaces (`:31-33`). Its enforcement
section (`:35-39`) says to report by "contacting the project team" and names no
address. There is no address to name: `contact@softmata.dev` at `README.md:648`
is the only email in any markdown file in this repository, and it is published
for hardware feedback (`README.md:644-648`), not as a conduct channel. Until
someone decides on a real reporting address, open an issue or contact a
maintainer directly. This gap needs closing properly rather than by pointing at
an address meant for something else.

## License

By contributing, you agree that your contributions will be licensed under the
Apache License 2.0, and that they are subject to the
[Contributor License Agreement](.github/CLA.md) — see step 7 of the PR process
for how to signal agreement.

---

## Further reading

This file covers the process. The rest of the contributor documentation lives
under `docs/`, and each of these answers a question this file deliberately does
not:

| Document | Answers |
|---|---|
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | Where does my change go? A crate and module map, and the execution model that decides which thread a node runs on. |
| [`docs/TESTING.md`](docs/TESTING.md) | Which suite does my test belong in, what does CI actually invoke, and what is committed but never reached? |
| [`docs/SHM_WIRE_FORMAT.md`](docs/SHM_WIRE_FORMAT.md) | I need to change the shared-memory layout. What else has to move with it? |
| [`docs/RELEASING.md`](docs/RELEASING.md) | How is a release cut, and what gates does the tag then face? |
| [`horus_net/docs/BLUEPRINT.md`](horus_net/docs/BLUEPRINT.md) | The network protocol spec that thirteen modules cite by section number. |
| [`horus_core/README.md`](horus_core/README.md) | An index into the crate's design documentation, most of which `cargo doc` does not render. |

For using HORUS rather than changing it, see the documentation site, and in
particular its Development section for the CLI, the environment variables, and
the runtime extension points.

---

Questions? Open an issue or start a discussion on GitHub.
