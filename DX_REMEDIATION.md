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
| `ed12cc9` | Failed ticks never counted; a node failing every tick reported Healthy | 1 (extended) |
| `78ad724` | `horus deploy` excluded its own binary; built the wrong manifest; could not expand `~` | 11 |
| `ee0dfca` | `--net` was a no-op; `horus run` escalated to sudo with no prompt | 6 |

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

1. **`.rate()` silently implies hard real-time.** Every plain Python node is
   classified `Rt`, given an auto-derived budget of 80% of its period, and
   isolated — permanently stopped — for exceeding it. The docs state the
   inverse rule twice. `BestEffort` is unreachable from Python (`rate=None`
   raises a raw `TypeError`, `rate=0` is rejected).

   Note the safer shape: rather than reclassifying (breaking for anyone
   relying on `.rate()` giving RT), consider keeping the RT thread but making
   the *auto-derived budget* opt-in. The isolation is the harm, not the
   scheduling.
2. **Safe-state is dispatched onto the stalled thread.** For the canonical
   watchdog case — a hung node — it can never run, while the log says "safing
   requested" and then goes quiet. A roboticist reads that as "the motors were
   stopped". Say what is true, and add a scheduler-level fallback that runs off
   the stalled thread.
3. **Python nodes cannot implement `enter_safe_state()`** at all — the binding
   forwards only `init`/`tick`/`shutdown`, so the hook the degradation ladder
   terminates in is a no-op for every Python node.
4. **`horus setup-rt` installs a Debian package name on Ubuntu**
   (`linux-image-rt-amd64`; Ubuntu's is `linux-image-realtime`), swallows the
   failure, prints "Setup complete", and tells you to reboot into an RT kernel
   you do not have. It also has no confirmation and no dry-run before
   `sudo apt install` of a kernel.
5. **Recording captures no payload for RT or C++ nodes** — 2,271 snapshots,
   zero bytes — and loses everything on an abnormal exit.
6. **Wire `npm run verify:code` into docs CI.** The harness already exists in
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
- Tests that `set_current_dir` must take `crate::CWD_LOCK`. A private lock only
  serialises a module against itself, which is how two unrelated `config` tests
  started failing; the deploy tests were rewritten to take an explicit root
  instead so they do not chdir at all.
- Two fixes in this branch were wrong on the first attempt and are recorded in
  their commit messages rather than quietly amended: re-admitting the deploy
  binary with rsync `--include` rules (pulled the whole 367 MB tree back in),
  and routing `--net` through `--features` (cargo rejects it — that flag
  applies to the user's crate, not the dependency).
