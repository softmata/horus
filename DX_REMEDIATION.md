# DX remediation — status

Working branch: `dx/remediation`. Every item below is committed with tests and
verified end to end against a release build.

## Landed

| Commit | Fixes | Tests |
|---|---|---|
| `0004df3` | Errors printed twice; `horus.toml` typos silently accepted; missing-field reported as a syntax error | 20 |
| `bf75961` | `message!` recursion trap on four natural syntax mistakes | 5 (trybuild) |
| `cc78d2c` | RT degradation invisible; cargo warnings discarded; `HORUS_SIM_MODE=0` enabled sim; health count underflow; negative param values rejected | 4 |
| `55e016e` | Panics never reached the blackbox; blackbox unreachable from a subdirectory | 5 |

### What each one was

**Errors printed twice.** `HorusError::Contextual` rendered
`{message}\n  Caused by: {source}` unconditionally, so `.horus_context("Cargo
build failed")` on an error that already displayed as `Cargo build failed`
printed both. The multi-line `Solutions:` block on `horus run` in an empty
directory appeared in full, twice. Display-only fix — `source()` is untouched,
so programmatic callers still see the whole chain.

**`horus.toml` typos.** No `deny_unknown_fields`, so serde discarded unknown
keys and `horus check` reported `* manifest valid`, exit 0. New
`manifest_lint` module reads the keys back off the source text and reports
them with line numbers. Warnings rather than errors on purpose: a hard error
would break every existing manifest with a stray key on upgrade — including
this repo's own `language = "rust"` — and there is no changelog for a user to
discover why. The warning states the 0.3.0 promotion.

Fuzzy matching alone was not enough: `language` is not a misspelling of
anything, so Levenshtein stayed silent on the most common real mistake in the
tree. `KNOWN_NON_FIELDS` gives those a tailored explanation instead of a guess.

**`message!` recursion trap.** The entry rule `($($input:tt)+)` also matched
the macro's own `@munch` output, so any syntax the structured arms rejected
recursed until the limit and reported `recursion limit reached` with a `help:`
that only made the compiler spin longer. Hit by `pub` on a field, a missing
comma, a semicolon separator, and a tuple struct. A tt-muncher needs a failure
arm; this one had a loop.

**RT degradation invisible.** SCHED_FIFO denied, pinning failed, governor stuck
on powersave, the ladder isolating a node, safe-state entered — all gated on
`--verbose`. Default operation ran a "real-time" loop on SCHED_OTHER, unpinned,
with zero output. Now unconditional; per-tick budget and deadline misses stay
gated because at 1 kHz they would flood, and the ladder already reports the
sustained case.

**Cargo warnings discarded.** All three cargo invocations printed stderr only
inside `if !status.success()`. rustc writes warnings to stderr, so a successful
build threw them away — including `#[must_use]` on a dropped `NodeBuilder`,
which is the framework's own guard against a node that is configured and never
registered. Exit 0, no diagnostic, binary starts with zero nodes.

**Panics never reached the blackbox.** `BlackBoxEvent::NodeError` had no call
site outside blackbox.rs's own tests, so the flight recorder could not record a
crash. Now written from both panic paths, before the `on_error` callback runs
so a panic there cannot cost us the record.

## Next, in order

Ranked by consequence per unit of work. Items 1–4 are one theme: the safety
story is the least-tested part of the framework.

1. **Increment `failed_ticks`/`errors_count` in the executors' panic branches.**
   A node panicking every tick reports `Health: Healthy, Errors: 0, Total
   Ticks: 0`. `record_tick_failure` exists and is correct; only the main-thread
   path calls it. Same code sites as the blackbox fix above.
2. **`horus deploy` excludes the binary it built.** `--exclude target` also
   matches `.horus/target/`. 37 KB of source ships and it prints "Deployment
   complete!". Two more fatal defects on the same path: deploy shells out to
   raw `cargo build` in a directory with no `Cargo.toml`, and `--run` builds
   `cd '~/horus_deploy'` with the tilde inside single quotes.
3. **`.rate()` silently implies hard real-time.** Every plain Python node is
   classified `Rt`, given an auto budget, and isolated for exceeding it. The
   docs state the inverse rule twice. `BestEffort` is unreachable from Python.
4. **Safe-state is dispatched onto the stalled thread.** For the canonical
   watchdog case — a hung node — it can never run, while the log says "safing
   requested". Say what is true, and add a scheduler-level fallback.
5. **`horus setup-rt` installs a Debian package name on Ubuntu**, swallows the
   failure, prints "Setup complete", and tells you to reboot into an RT kernel
   you do not have.
6. **`horus run --net` is a no-op** — the `net` feature is never enabled, so
   `horus_net` is unreachable from the documented path.
7. **`horus run` executes `sudo apt install -y` with no prompt.** The prompt to
   reuse already exists in `registry/helpers.rs`.
8. **Wire `npm run verify:code` into docs CI.** The harness already exists in
   the docs repo and would mechanically catch the large set of documented APIs
   that do not exist.

## Notes for whoever picks this up

- `cmake_gen::tests::real_cmake_*` and `commands::action::tests::discover_actions_*`
  fail on pristine `main` too — missing eigen/fmt/gtest, and shared-memory
  state flakiness under parallel load. Not caused by this branch.
- `scheduling::scheduler::tests::test_ready_dispatch_single_node_no_overhead`
  is a timing test and fails under load; it passes on a quiet machine.
- The trybuild snapshots in `horus_core/tests/ui/` regenerate with
  `TRYBUILD=overwrite cargo test -p horus_core --test ui`.
- `manifest_lint`'s key lists are a hand-maintained second copy, so
  `manifest_lint_covers_all_manifest_fields` round-trips a populated
  `HorusManifest` and fails the build if a field is added without listing it.
