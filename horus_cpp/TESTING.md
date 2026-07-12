# horus_cpp Production Testing

Last updated: 2026-04-23. This document describes the full test surface of
`horus_cpp`. It is the canonical source for what is tested, how, where the
evidence lives, and what gaps remain.

## Summary

| Dimension                    | Status                | Evidence                                              |
|-----------------------------|-----------------------|-------------------------------------------------------|
| Rust unit tests (horus_cpp)  | 139                   | `cargo test -p horus_cpp`                             |
| Rust unit tests (macros)     | 77                    | `cargo test -p horus_cpp_macros`                      |
| C++ gtest count              | 61                    | `ctest --test-dir target/cpp_tests -N`                |
| Proptest properties          | 4 × 256 cases = 1024  | `tests/proptest_ffi.rs`                               |
| Loom model tests             | 1                     | `loom_tests/tests/loom_scheduler.rs`                  |
| libFuzzer targets            | 4                     | `fuzz/fuzz_targets/`                                  |
| AddressSanitizer (ASan)      | CI-gated              | `cpp-asan` job                                        |
| ThreadSanitizer (TSan)       | CI-gated              | `cpp-tsan` job                                        |
| UBSanitizer                  | CI-gated              | `cpp-ubsan` job                                       |
| Valgrind memcheck            | CI-gated              | `cpp-valgrind` job                                    |
| Line coverage (Rust floor)   | 80% enforced          | `cargo llvm-cov --fail-under-lines 80`                |
| Line coverage (C++ floor)    | 70% enforced          | lcov + awk threshold in `cpp-coverage`                |
| Nightly soak duration        | 2 hours               | `cpp-bindings-nightly.yml` → `cpp-soak`               |
| Fault-injection scenarios    | 5                     | `tests/cpp_fault_injection.cpp`                       |
| Cross-process Service        | 100 req/resp reliable | `cross_process_svc_client` → 99% success threshold    |
| Cross-process IPC (CmdVel)   | Yes                   | `cpp-cross-process` CI job                            |
| Cross-language (Rust↔C++↔Py) | Yes                   | `cpp-cross-language` CI job                           |
| Fuzz long-run (nightly)      | 30 min × 4 targets    | `cpp-bindings-nightly.yml` → `cpp-fuzz-long`          |

## Test Matrix — Per-PR (.github/workflows/cpp-bindings.yml)

| CI job                | Runs                                                                       |
|-----------------------|----------------------------------------------------------------------------|
| `rust-ffi-tests`      | 139 Rust #[test]s + proptest (4 × 256 cases) + loom model (LOOM_MAX_PREEMPTIONS=3) |
| `cpp-compile-test`    | cmake + ctest: 61 gtests, JUnit artifact uploaded                          |
| `cpp-cross-process`   | CmdVel IPC, JSON IPC, Service 100 req/resp × 99% threshold, Action         |
| `cpp-cross-language`  | C++↔Python cross-language via `cross_lang_tri_test.sh`                     |
| `cpp-asan`            | cpp_stress_test + cpp_full_api_test with -fsanitize=address                |
| `cpp-tsan`            | cpp_stress_test + cpp_ergonomic_e2e with -fsanitize=thread (nightly Rust + -Zbuild-std -Zsanitizer=thread) |
| `cpp-ubsan`           | cpp_full_api_test + cpp_ergonomic_e2e with -fsanitize=undefined,integer    |
| `cpp-valgrind`        | cpp_ergonomic_e2e under valgrind --tool=memcheck (30-min timeout)          |
| `cpp-fuzz`            | 4 libFuzzer targets × 60 s each; crashes uploaded as artifact              |
| `cpp-coverage`        | cargo llvm-cov + lcov merged, uploaded to Codecov flag `horus_cpp`         |
| `cpp-benchmark`       | cpp_benchmark binary + `cargo bench`                                       |
| `cpp-success`         | Gate — all jobs above must pass                                            |

## Test Matrix — Nightly (.github/workflows/cpp-bindings-nightly.yml)

Schedule: `0 2 * * *` (daily 2 AM UTC). Also runnable via workflow_dispatch.

| CI job                 | What it runs                                                                  |
|------------------------|-------------------------------------------------------------------------------|
| `cpp-soak`             | `cpp_soak_test` — 100 Hz tick + pub/sub for 2 h; samples VmRSS/threads/fds every 60 s; fails on >10% RSS growth from t+300s baseline |
| `cpp-fault-injection`  | 5 TESTs: SubscriberKilledMidPublish, PublisherHoldsLoanThenKilled, ShmNamespaceMissing, AbiVersionContract, RapidRestart100Cycles |
| `cpp-fuzz-long`        | 4 libFuzzer targets × 30 min each                                             |

## Sanitizer Coverage

- **ASan** — heap/stack buffer overflow, UAF, double-free. Gated per PR.
- **TSan** — data races across FFI threads. Built with `-Zbuild-std -Zsanitizer=thread` (Rust) + `-fsanitize=thread` (C++). Suppressions at `horus_cpp/tsan.supp` if needed (none at present).
- **UBSan** — integer overflow, misalignment, null deref, enum-out-of-range. Flags: `-fsanitize=undefined,integer -fno-sanitize-recover=all`. `UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1:abort_on_error=1`.
- **Valgrind memcheck** — uninitialized reads, SHM-region errors ASan misses. Suppressions at `horus_cpp/valgrind.supp` if needed (none at present).
- **MIRI** — inherited from `.github/workflows/safety.yml`; weekly on `horus_core`. Not applied to `horus_cpp` (CXX bridges outside MIRI's model).

## Coverage Floor

- Rust: ≥ 80% line coverage enforced via `cargo llvm-cov --fail-under-lines 80`.
- C++: ≥ 70% line coverage enforced via awk threshold on `lcov --summary`.
- Rationale: C++ paths include some unreachable-by-unit-test branches (cross-process drivers, error fallbacks) that are exercised in cross-process jobs but not merged into the main binary's counters. The 10-point gap reflects that structural limitation.
- Reports: uploaded to Codecov with flag `horus_cpp` and as a GHA artifact (`rust-lcov.info`, `cpp-lcov.info`, `merged-lcov.info`).

## Soak + Fault Injection

### Soak — `tests/cpp_soak_test.cpp`

- Default duration 2 h (configurable via `HORUS_SOAK_DURATION_SEC`).
- 100 Hz scheduler, 1 publisher, 1 subscriber, loan-based publish in tick callback.
- Samples `/proc/self/status:VmRSS`, thread count, fd count every 60 s.
- Baseline at t+300 s (post-warmup).
- Fails on: RSS growth > 10%, thread delta > 2, fd delta > 2 vs baseline.

### Fault Injection — `tests/cpp_fault_injection.cpp`

5 scenarios, all verified locally (245 ms total):

1. `SubscriberKilledMidPublish` — subscriber SIGKILL'd mid-run; publisher survives.
2. `PublisherHoldsLoanThenKilled` — publisher dies holding a loan; new subscriber recovers.
3. `ShmNamespaceMissing` — non-existent namespace; publish must not segfault.
4. `AbiVersionContract` — `horus_get_abi_version()` ≥ 1 and matches header.
5. `RapidRestart100Cycles` — 100 scheduler new/destroy with publish in each.

## Known Gaps (v1.0 exit criteria)

- [x] GoogleTest framework adopted — `tests/CMakeLists.txt`, 61 gtests, cmake+ctest in CI
- [x] TSan CI job — `cpp-tsan` in cpp-bindings.yml
- [x] UBSan + Valgrind CI jobs — `cpp-ubsan`, `cpp-valgrind`
- [x] libFuzzer harness — 4 targets in `fuzz/fuzz_targets/`
- [x] proptest + loom — 4 properties + 1 loom model
- [x] llvm-cov with 80% / 70% floor — `cpp-coverage` job
- [x] Nightly soak + fault injection — `cpp-bindings-nightly.yml`
- [x] Service FFI cross-process reliability — 100 req/resp × 99% threshold
- [x] Pool leak-free cycles — PointCloud and Tensor verified; Image **known bug** (see below)
- [x] Evidence pack — this document

### Known Production Bugs

- **Image pool slot leak**: `horus_image_destroy` does not release the pool slot. After 18 alloc/destroy cycles on a 64-slot pool, subsequent allocations return NULL. Regression guard lives at `cpp_full_api_test.cpp::Pool.ImageAllocReleaseCycleLeakFree` (currently `GTEST_SKIP` with FIXME). Fix is in `pool_ffi.rs` or `horus_core::Image::Drop`. **Filed as a blocker for v1.0.**

## Not In Scope (deferred)

- **MISRA-C++ static analysis** (clang-tidy with misra-* checks) — tracked externally.
- **ISO 26262 / IEC 61508 / DO-178C certification trail** — requires tool-qualified toolchain; not pursued in this roadmap.
- **Branch coverage reporting** — line coverage is the current floor; branch coverage is a future phase.
- **Typed (monomorphized) service FFI macro `impl_service_ffi!`** — existing `Topic<T: Pod>` gives typed pub/sub; macro follow-up awaits a first consumer.
- **Full cross-process pool streaming (Image/PointCloud/Tensor pub/sub binaries)** — pool types aren't directly publishable (handles, not Pod); wrapper-message protocol deferred to follow-up.
- **Latency benchmark comparing JSON service vs typed** — no typed service shipped, so benchmark deferred.

## Running Locally

Headless build:

```bash
CARGO="$HOME/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo"
"$CARGO" test --no-default-features -p horus_cpp
```

Full C++ gtest suite:

```bash
"$CARGO" build --no-default-features -p horus_cpp
cmake -S horus_cpp/tests -B target/cpp_tests -DHORUS_RUST_TARGET_DIR=$(pwd)/target/debug
cmake --build target/cpp_tests -j
LD_LIBRARY_PATH=target/debug ctest --test-dir target/cpp_tests --output-on-failure
```

Proptest:

```bash
"$CARGO" test --no-default-features -p horus_cpp --test proptest_ffi -- --test-threads=1
```

Loom (separate crate):

```bash
cd horus_cpp/loom_tests
RUSTFLAGS="--cfg loom" LOOM_MAX_PREEMPTIONS=3 "$CARGO" test --release
```

Sanitizers (one at a time):

```bash
# ASan
RUSTFLAGS="-Zsanitizer=address" "$CARGO" +nightly build -Zbuild-std \
  --target x86_64-unknown-linux-gnu --no-default-features -p horus_cpp
# Then compile C++ with -fsanitize=address and run.
```

Fuzz (one target):

```bash
cd horus_cpp
"$CARGO" +nightly fuzz run fuzz_json_service -- -max_total_time=60
```

Soak (short, local smoke):

```bash
LD_LIBRARY_PATH=target/debug HORUS_SOAK_DURATION_SEC=600 \
  target/cpp_tests/cpp_soak_test
```

## Changelog

- 2026-04-23: v1.0 — production testing foundation complete. Roadmap `roadmap-mob144fu-f6q33s` closed with one known blocker (Image pool leak).
