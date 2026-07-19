# HORUS install → first-run E2E audit (2026-07-18)

Baseline: clean tree @ `aa33d470`, branch `main`.

## Build health: GOOD
All three configs that install/CI/docs actually invoke pass clean:
- `cargo check --workspace --all-targets` (default features) — OK
- `cargo check --no-default-features` (CLAUDE.md headless) — OK
- `cargo build --release -p horus_manager --no-default-features` (install.sh's exact cmd) — OK, 55s

The build is not broken. Every break is in the **install → first project** path.

---

## F1 — CRITICAL: install.sh source-build path clones a branch that does not exist
`install.sh:206` → `git clone --depth 1 --branch "$BRANCH"` with `BRANCH="release"`.
`git ls-remote --heads origin` returns only `main`, `dev`, `fix/net-estop-sched-h1`.
**No `release` branch exists.** The clone fails → install aborts.

## F2 — CRITICAL: prebuilt-binary fast path 404s for every platform
`install.sh` tries `releases/latest/download/horus-linux-amd64.tar.gz` first.
Verified HTTP 404. Latest release (v0.2.0, id 300155541) has **zero assets**.
`.github/workflows/release.yml` *does* define the matrix that would build
`horus-{linux,macos,windows}-{amd64,arm64}` — it has evidently never
completed successfully for a tag. Tag `v0.2.1` has no GitHub Release at all.

**F1 + F2 together mean install.sh fails 100% of the time on every platform.**

## F3 — CRITICAL: install.sh deletes the horus source that `horus run` requires
Rust projects do not depend on a published crate. `cargo_gen.rs:107` calls
`find_horus_source_dir()` (`run_rust.rs:923`), which searches `$HORUS_SOURCE`,
`/horus`, `~/softmata/horus`, `~/horus`, `/opt/horus`, `/usr/local/horus`, then
a `~/.horus/cache/horus@0.1.0` fallback. `write_horus_path_deps` emits **path
dependencies** into `.horus/Cargo.toml`.

install.sh clones to a `mktemp -d`, builds, copies the binary, then
`rm -rf "$CLONE_DIR"` (line ~247). **The source it needs is deleted.**
`horus` is not on crates.io either, so there is no fallback.

Result: a correctly-installed user has a working `horus` binary and no way to
build a Rust project.

## F4 — HIGH: cargo_gen swallows the "source not found" error, producing a baffling failure
`cargo_gen.rs:107`:
```rust
if let Ok(horus_source) = crate::commands::run::find_horus_source_dir() {
    write_horus_path_deps(&mut cargo, &horus_source, manifest);
}
```
`find_horus_source_dir()` has a *good* actionable error message (3 numbered
remedies). `if let Ok` discards it. `[dependencies]` is written empty and the
user instead sees 9 rustc errors:
`cannot find type Topic / CmdVel / Scheduler`, `Result expected 2 generic args`.

**Reproduced end-to-end** in an isolated HOME — see "Repro" below.

## F5 — HIGH: `horus new -r` template uses `CmdVel`, which is not in the prelude
`new.rs:443` and `new.rs:475` (both macro and non-macro templates) use
`CmdVel::new(1.0, 0.0)`. `horus/src/lib.rs:425-427` states explicitly:

> NOTE: TransformFrame and robotics messages are no longer in horus core.
> Use `horus_robotics::prelude::*` for robotics messages (CmdVel, Imu, etc.)

The template was never updated after the decomposition. Even with F3/F4 fixed
and `horus` correctly on the dep path, the generated project still fails to
compile on `CmdVel`.

## F6 — MEDIUM: stale hardcoded version in the cache fallback
`run_rust.rs:954` → `cache_dir.join("horus@0.1.0")`. Actual workspace version is
`0.2.0`. The cache escape hatch can never hit for a current install.

## F7 — MEDIUM: uninstall.sh misses the binary when install used the fallback dir
`install.sh:find_install_dir()` prefers `~/.cargo/bin`, else `~/.local/bin`.
`uninstall.sh:134` hardcodes `INSTALL_DIR="$HOME/.cargo/bin"`.
A user without Rust (exactly the prebuilt-binary user) gets the binary in
`~/.local/bin`, and uninstall silently leaves it there.

## F8 — MEDIUM: uninstall.sh never removes the Linux config dir (leaves credentials)
`horus_sys/platform/mod.rs:208 config_dir()` → `$XDG_CONFIG_HOME/horus`,
i.e. `~/.config/horus` on Linux. It holds `auth.json` (`paths.rs:59`) and
`workspaces.json`. uninstall.sh handles `~/.horus`, macOS Application Support
and Windows AppData — **but has no Linux `~/.config/horus` branch.**
Uninstall leaves stored credentials on disk.

## Non-findings (checked, fine)
- `horus env --init` — install.sh calls it; it exists as a hidden subcommand and works.
- uninstall.sh `rm -rf $CARGO_REGISTRY` (line 589) — gated behind an explicit
  y/N prompt with a clear "affects all Rust projects" warning. Acceptable.
- `CARGO_TARGET_DIR` pointing at pangroll — artifact of this shell env, not a horus bug.

---

## Repro (isolated HOME, no writes to the real one)
```
FAKEHOME=<scratch>/fakehome
HOME=$FAKEHOME XDG_CONFIG_HOME=$FAKEHOME/.config horus new my_robot -r   # OK
cd my_robot && horus build
# → 9 errors: cannot find type `Topic`/`CmdVel`/`Scheduler` in this scope
# → .horus/Cargo.toml has an empty [dependencies] section
```

## Severity summary
The build is healthy. The *product* is not reachable by a new user: install
fails (F1+F2); if it succeeded the user still could not build a project (F3+F4);
if that were fixed the template still would not compile (F5).

---

# Resolution (branch `fix/e2e-install-first-run`, commit cbdd0d8d)

| # | Status | Fix |
|---|--------|-----|
| F1 | FIXED | `BRANCH` defaults to `main` (`HORUS_INSTALL_BRANCH` overrides) |
| F2 | **NOT FIXED — needs CI** | install.sh logic now degrades gracefully to a source build, but publishing the assets requires a successful `release.yml` run. Cannot be verified from here. |
| F3 | FIXED | Source cached at `~/.horus/cache/horus@<version>`; prebuilt binary now only skips the compile step |
| F4 | FIXED | Error propagated with `.context(...)` instead of `if let Ok` |
| F5 | FIXED | Templates use `Twist::new_2d()`; test retargeted + negative assertion |
| F6 | FIXED | Cache lookup prefers `horus@<CARGO_PKG_VERSION>`, falls back to any `horus@*` |
| F7 | FIXED | uninstall checks `~/.cargo/bin` **and** `~/.local/bin` |
| F8 | FIXED | uninstall removes `$XDG_CONFIG_HOME/horus` on Linux/BSD |
| F9 | FIXED | Python template uses `node.recv()`; regression test added |
| F10 | FIXED | Two cache roots existed (`~/.cache/horus` XDG vs `~/.horus/cache`); both are now searched |

F9/F10 were found while verifying the F3/F5 fixes:
- **F9** — the Python template called `node.get()`, which does not exist
  (`horus_py/horus/__init__.py` has recv/recv_all/has_msg/send). It sat behind
  `if node.has_msg(...)`, so it raised AttributeError only once a publisher
  appeared — a project that looks fine until it isn't.
- **F10** — `paths::cache_dir()` resolves to `~/.cache/horus` (XDG), but
  install.sh/uninstall.sh and `horus clean` manage `~/.horus/cache`. The
  installer wrote one and the resolver read the other.

## Verification performed
Clean-room, isolated `HOME`, source present *only* in the cache (no dev checkout):
1. `horus new bot -r` — OK
2. template contains `Twist`, zero `CmdVel` occurrences
3. `horus build` — generates correct path deps, compiles
4. binary produced at `.horus/target/debug/bot`
5. `horus node list` — node Running
6. `horus topic list` — `motors.cmd_vel`, 269 msgs, 88 Hz, active
7. `horus topic echo motors.cmd_vel` — `Type: Twist`, payload
   `00 00 00 00 00 00 f0 3f` = `1.0f64` = `Twist::new_2d(1.0, 0.0)`
8. `uninstall.sh` in the same isolated HOME: removed the binary from the
   `~/.local/bin` fallback and the credential file; correctly kept the cargo
   registry. Real `$HOME` verified untouched afterwards.

Tests 3080 passed / 0 failed. Clippy clean. `bash -n` clean on both scripts.
horus-docs `validate-docs.js`: 278 pages, 0 errors.

## Known remaining issues (NOT fixed)
- **F2 / release.yml** — no GitHub Release has ever carried binaries. Until a
  tagged run of `release.yml` succeeds, every install compiles from source
  (~1-5 min). Needs a CI run to verify; not verifiable in this environment.
- **`horus node list` TICKS always 0** — cosmetic. The node ticks correctly
  (proved by 88 Hz / 269 msgs on the topic), but the live-count override at
  `discovery/nodes.rs:114-130` never fires, so the column stays 0. Pre-existing,
  unrelated to these fixes.
- **Python e2e not fully driven** — `horus run` on a Python project resolves
  deps via `pip`, which is absent in this sandbox (`No module named pip`). The
  template fix (F9) is verified by test and inspection; the full Python
  new→run journey was *not* executed end to end.
- **Windows support claim** — `installation.mdx` lists Windows as fully
  supported. Left unchanged: `horus_sys` has real Windows implementations
  (`CreateFileMappingW`), `release.yml` builds `x86_64-pc-windows-msvc`, and
  `reference/platform-support.mdx` is self-consistent. Flagging only because a
  project note claims horus does not compile on Windows — that note appears
  stale, but I could not compile-verify Windows here.

---

# Round 2 — full documented-journey conformance (2026-07-18)

Goal: behave like a user reading horus-docs and make every documented journey
real, not just the install path. Verified against a frozen substrate: source
cached at `~/.horus/cache/horus@0.2.0` from branch HEAD, isolated `HOME`,
`PYTHONUSERBASE=/home/neos/.local`, shared warm `CARGO_TARGET_DIR`, per-run
`HORUS_NAMESPACE` for shm isolation.

## C1 — CRITICAL: the entire C++ journey was unreachable (FIXED, `ea83e2af`)
`horus new --cpp` scaffolds `src/main.cpp`, but:
- `detect_language()` (`run/deps.rs:142`) mapped only `.rs`/`.py`. A test
  asserted `.cpp` *errors*, locking the gap in.
- `auto_detect_main_file()` (`run/mod.rs:757`) listed only `main.rs`/`main.py`,
  so `horus build` failed "No main file detected" — and the remedy text did not
  mention C++ at all.
- `build_project()` had no `"cpp"` arm and bailed "Unsupported language: cpp",
  though the two run paths below it already dispatched to `build_cpp`.

The C++ pipeline (`run_cpp.rs`, `cmake_gen.rs`) was fully implemented and simply
could not be reached. Docs (`getting-started/cpp.mdx`) promise
`horus new my-robot --cpp && horus run` works, so: DOCS_ARE_RIGHT_CODE_WRONG.

## C2 — CRITICAL: C++ projects could not link the bindings (FIXED, `acf90f8e`)
Even once reachable, `#include <horus/horus.hpp>` failed. Nothing wired
horus_cpp's headers or library:
- `horus new --cpp` wrote a **root** `CMakeLists.txt`, which (a) violates the
  documented convention that native build files live in `.horus/` and never the
  project root, (b) *shadows* the generated one (`run_cpp` prefers a root file),
  and (c) used `find_package(horus QUIET)` — which never resolves, so it fell
  through to "building without horus bindings" while `main.cpp` includes the
  header unconditionally.
- `cmake_gen` only added the project's own `../include`.

Fix: stop scaffolding the root file; `cmake_gen` emits wiring guarded on
`HORUS_CPP_INCLUDE`/`HORUS_CPP_LIB`; `run_cpp::ensure_horus_cpp()` resolves the
source tree, builds the `horus_cpp` staticlib on demand, and passes both via
`-D` so host paths stay out of the generated file.

## C3 — CRITICAL: C++ `Scheduler::spin()` ignored every rate setting (FIXED, `acf90f8e`)
`scheduler_impl.hpp` busy-looped on `horus_scheduler_tick_once()`:

```cpp
while (is_running()) { horus_scheduler_tick_once(inner_); }
// "For now, spin = repeated tick_once. Real spin() would call scheduler_run()."
```

`scheduler_run` existed in Rust (`scheduler_ffi.rs:115`) but was never exposed
through the C API, so C++ had no way to reach the real run loop.

**Measured**: a node declaring `tick_rate(100_hz)` published 334,721 messages in
3s = **~111,000 Hz**, pinning a core at ~100% CPU. For a real-time robotics
framework driving hardware, a control loop running 1000x its declared rate is a
safety-relevant defect, not a performance nit.

Fix: expose `horus_scheduler_run()`, declare it in `horus_c.h`, and have
`spin()` delegate to the Rust run loop (which honors tick rate, per-node rates,
budgets and miss policies).

**After**: ~90 Hz measured for a 100 Hz config (matching Rust's 88 Hz), 1.4% CPU.

### Verified C++ journey (end to end, clean project)
`horus new finalbot --cpp` -> `horus build` (3.1s warm) -> run -> `horus topic
list` shows `cmd_vel` at ~90 Hz -> `horus topic echo cmd_vel` prints
`linear: 0.300, angular: 0.000`, exactly what the template publishes.

### C++ caveat
The `spin()` fix has **no automated regression guard**. A rate assertion needs a
C++ runtime test, and the C++ gtest suite is not part of the Rust test gate.
Nothing would catch a reintroduction of the busy-loop except the comment left in
the header. Worth a follow-up runtime test.

## Corrections — two observations that did NOT reproduce
Recorded so neither is mistaken for a real defect:
- **`horus topic echo` on C++ topics**: appeared to print nothing. It was stdout
  block-buffering under `| head` combined with a `timeout` kill. Writing to a
  file shows it works correctly and decodes `CmdVel` field-wise.
- **Python "No module named pip"** (reported in Round 1 as a sandbox limit): the
  real cause was my isolated `HOME` hiding pip, which lives in the real
  `~/.local`. With `PYTHONUSERBASE` set, pip and `import horus` both work, so the
  Python surface *is* verifiable here. Round 1's "Python e2e not fully driven"
  caveat was overstated.

---

# Round 3 — full documented-journey conformance sweep (2026-07-19)

Drove *every runnable instruction* in horus-docs against the frozen substrate
(17-chunk extract+verify workflow: 3488 code blocks seen, 488 classified
runnable, 111 reported failures, 59 confirmed after verification). Fixes below;
the bulk (~50 API-drift items) are documentation corrections applied by a
separate fixing pass. The genuine **product** bugs the sweep surfaced:

## R1 — CRITICAL: RT nodes ran at half their configured rate (FIXED, `da159311`)
The headline product bug. Every RT node (`.rate()`/`.budget()`/`.deadline()`)
ticked at exactly half its configured frequency. `rt_executor.rs` samples
`loop_start` before `last_tick` is stamped (inside tick_node) and sleeps exactly
`tick_period`, so the strict `elapsed < period` gate rejected every on-time
boundary tick — the node fired only every other loop. Fix: accept within half a
loop period (`elapsed < period - tick_period/2`). Verified: single 100 Hz node
→ 100.0 Hz (was ~50); mixed 100+20 Hz → 99.9 + 20.0 Hz (slow node not
over-fired). Added a non-flaky regression test. **This is the shared root cause
of the earlier "Python ~55% of rate" observation** — the Python RT path uses the
same executor.

## R2 — HIGH: multi-node Python launches showed no output (FIXED, `14244690`)
`horus run "nodes/*.py"` pipes each child's stdout to prefix `[node]`; CPython
full-buffers a piped stdout, so the documented live per-node output never
appeared (for a long-running node, never). Fix: `PYTHONUNBUFFERED=1` on piped
children. Verified: application output now streams line-by-line.

## R3 — HIGH: `horus msg` and `horus deps` broken on any install (FIXED, `0a186a61`)
- `horus msg list/info/hash` searched only `horus_library/messages` — a crate
  removed in 7b430279. Could never succeed. Now resolves `horus_types/src` via
  the same source resolution as everything else.
- `horus deps tree/why/outdated/audit` ran cargo from the project root (no
  Cargo.toml by design — it's in `.horus/`). Now runs from `.horus/`.

## R4 — MEDIUM: `horus record export --format json` dropped all payloads (FIXED, `a25ea817`)
JSON export emitted metadata only (525 bytes for a 1193-tick recording), making
the documented "script the export" use case impossible, while the CSV export
already carried per-tick payloads. Added the `snapshots` array (tick,
timestamp_us, inputs, outputs; bytes hex-encoded). Verified against a 253-tick
recording.

## R5 — SERIOUS, NOT FIXED: concurrent same-namespace scheduler startup segfaults
Two schedulers starting concurrently in the same HORUS_NAMESPACE race in the SHM
layer and **one segfaults** (reproduced with two plain `python3 node.py`;
different namespaces are fine). This breaks the documented multi-node Python
pattern (`horus run "nodes/*.py"`). It is NOT the control topic (created with
`.ok()`, graceful). Root cause is a TOCTOU race in concurrent SHM
namespace/topic/registry creation during startup. Left for a proper debugging
session (ASAN/valgrind under two concurrent starts) rather than a blind patch —
see task tracker. **A memory-safety bug in a safety-critical framework; high
priority for follow-up.**

## R6 — HIGH: the canonical `message!` macro produced non-compiling projects (FIXED, `8753a7b3`)
Root cause of the whole "cannot find derive macro Serialize" cluster. The
`message!` macro (horus's advertised "zero configuration" way to define a
message) expands to `serde::Serialize`/`serde::Deserialize`, which need serde as
a **direct** dependency — but cargo_gen never injected it. A fresh `horus new -r`
project using `message!` failed with `error[E0433]: cannot find module or crate
serde`. Fix: cargo_gen injects `serde = { version = "1", features = ["derive"] }`
into generated projects (single-package and workspace), skipped if the user
declared it. Verified: a `message!` project now compiles and links (was broken);
default template still builds.

## Final acceptance test (fresh clone of the fixed branch, isolated HOME)
`horus new` → `build` → `run` → introspect, all three languages, source present
only in `~/.horus/cache`:
- **Rust**: builds; `motors.cmd_vel` active; a `.rate(50.hz())` RT node measures
  **49.8 Hz** (was ~25 before R1); `message!` project compiles + links.
- **C++**: builds (cmake + auto-built bindings); publishes `cmd_vel` (272 msgs).
- **Python**: `horus new -p` → `horus run` publishes `motors.cmd_vel` (187 msgs).
  ⚠️ measured 14 Hz for a 30 Hz template — this is the **prebuilt Python wheel**
  in the sandbox still carrying the old rt_executor; the R1 fix is in source and
  reaches Python only after a wheel rebuild (`maturin develop`). The Rust RT path
  (built from the fixed source) confirms the fix at 49.8/50 Hz.

---

# Round 4 — remaining open bugs resolved (2026-07-19)

The five follow-up items (#9–#13) that Round 3 recorded but did not fix. All
Python-binding fixes verified against a rebuilt wheel (`maturin develop` into an
isolated venv); all shm repros run against a cleaned `/dev/shm`.

## #10 — FIXED: Python Tensor topic round-trip returned garbage (`2b53c68c`)
`send` shipped the source descriptor unchanged, but `from_numpy` allocates in a
process-local scratch pool while the receiver reads the topic's shared pool —
`data_ptr()` came back null. Now `send` allocates in the topic pool
(`alloc_tensor`) and copies bytes in (the `send_handle` contract, as Image
already did); `recv` uses `recv_handle`/`from_owned`, which validates the
descriptor's `pool_id`. Fixing delivery exposed a latent borrow bug —
`Tensor.numpy()`/`torch()` held an immutable borrow across `numpy.asarray(slf)`,
which takes `borrow_mut` for `view_keepalive` → "Already borrowed" panic; scoped
the borrow. Verified: same-process and cross-topic f32/f64 round-trips match
exactly; Image still round-trips.

## #11 — FIXED: `DetectionList.append(horus.Detection(...))` rejected (`2b53c68c`)
Two `#[pyclass(name = "Detection")]` exist — perception's `PyDetection` (list
element) and messages' `PyDetectionMsg` (topic message, which wins the
`horus.Detection` name by registering last). `append` wanted the former; users
construct the latter. `append`/`from_list` now coerce either into the stored
type via a lossless field copy. Verified: append, `__getitem__`, `from_list`.

## #12 — FIXED: `horus record export` empty for RT nodes (`6e610d6c`)
The main-thread scheduler records each node's inputs per tick; the RT executor
only called `begin_tick`/`end_tick`. Since any `.rate()` node (all Python nodes,
default rate=30) runs on the RT executor, RT recordings held only metadata.
Added the same input capture to `tick_node`, gated on `is_active_tick()` (zero
cost when not recording). Verified: a 2-node `.rate(50)` project under
`.with_recording()` — subscriber captures 86/87 snapshots with the real Twist
payload; publisher captures none. Was 0.

## #13 — FIXED: `build_messages()` failed confusingly for pip users (`eb2c44be`)
It compiles a Rust extension via maturin, which needs the source crate; a wheel
install has no Cargo.toml at the package root. Now detects that and raises a
clear error pointing at the no-build dict-topic path. (Making compiled custom
messages work for wheel users — shipping the codegen crate in the wheel — is a
packaging decision left for the maintainer.)

## #9 — REFUTED: the "segfault" was binary version skew, not a race
Controlled comparison, identical nodes/namespace/concurrency, only the wheel
version varied:
- **Stale prebuilt `~/.local` wheel** (older horus_core, shm pool v3): one
  process **segfaults** every trial (2/2).
- **Wheel rebuilt from current HEAD** (matching horus_core, v4): clean every
  trial (3/3); `horus run "nodes/*.py"` with 3 nodes runs with 0 crashes.

So concurrent same-namespace scheduler startup is **not** a code race — the
crash was a shared-memory layout mismatch between mismatched binary versions in
the test environment (the same v3/v4 skew that manufactured 47 phantom test
failures in Round 3). With consistent builds the documented multi-node pattern
works. **Latent hardening follow-up** (separate, lower priority): a cross-version
shm attach segfaults instead of erroring cleanly — one path already guards
(`pool_registry.rs:67` → "expected 4, got 3"); some other startup shm structure
dereferences an old-format region without a version check. Locating it needs a
backtrace from a symbol-built old-version binary; not attempted rather than
guessed.

## Documentation corrections (code is authoritative; docs drifted)
~50 confirmed doc divergences, applied against the real API. Dominant clusters:
- Python message constructors are flattened scalars, not nested/array kwargs
  (`Twist(linear_x=…)` not `Twist(linear=[…])`; `Pose3D(x,y,z,qx…)`;
  `Imu(accel_x…)`; `Detection(class_name=…)`; `StereoInfo(...)` no `fx`).
- `horus.msggen`: real API is `register_message`, not the ghost
  `define_message`/`define_numpy_message` (7 pages).
- Python TransformFrame: `register_frame`/`tf`/`tf_at`/`wait_for_transform`, not
  `lookup`/`add_frame`; and the root frame must be registered before use (6 pages).
- Rust custom messages: use the `message!` macro, not manual `derive(Serialize)`
  under the prelude (serde isn't re-exported) — affected many tutorials/recipes.
- Rust robotics types (`CmdVel`/`Imu`) live in `horus_robotics`, not injected —
  recipes should use `Twist` from the prelude or declare the dep.
- `scheduler.run()` not `scheduler.build()?.run()`; `Topic(type, capacity)` not
  `Topic(name, type)`; `Node.recv(topic)` has no `timeout=`; no `node.state`;
  `[package]` not `[project]`; C++ examples need `#include <cmath>`; no
  `RouterClient`/`horus router`; `horus run` has no `--project`.

## Corrections — observations that did NOT reproduce (not defects)
- `horus topic echo` / `horus topic list` printing nothing for C++ topics: stdout
  block-buffering under `| head` + a `timeout` kill. Writing to a file shows both
  work (C++ `cmd_vel` decodes `linear: 0.300`).
- Python "No module named pip": the isolated test HOME hid the real `~/.local`
  pip; with `PYTHONUSERBASE` set the whole Python journey is verifiable.

## Correction to an earlier observation
An initial run of `horus topic list` printed "No active topics found" while the
node was up. This did **not** reproduce: with the same binary and a live node it
reports the topic correctly (item 6 above). The first observation was a timing
artifact, not a defect. Recorded here so it is not mistaken for a real finding.
