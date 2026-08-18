# Changelog

All notable changes to HORUS are recorded here.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
this project uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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

## Unreleased

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
- **python** — PEP 561 `py.typed` marker, so the shipped `_horus.pyi` stubs are
  actually used. Type checkers previously skipped the package entirely.

### Fixed

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
