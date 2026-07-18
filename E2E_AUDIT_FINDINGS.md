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
