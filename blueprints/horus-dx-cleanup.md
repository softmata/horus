# Horus DX: Make horus.toml Cargo-Grade

**Vision:** horus.toml is a professional-grade polyglot robotics manifest. One file, one CLI, any language. The generation layer (`.horus/`) is invisible. Students learn `horus` — not cargo, pip, cmake. Experienced devs get a unified workflow they can't get anywhere else.

**Why horus.toml, not Cargo.toml:** Cargo.toml is Rust-only. horus.toml manages Rust + Python + C++ in one place. `horus run` works for `.rs` and `.py`. `horus test` runs cargo test AND pytest. `horus add` adds deps regardless of language. No other tool does this.

## What Already Works (DO NOT rebuild)

These are fully implemented and functional:

- **Multi-language dispatch** (`dispatch.rs`) — auto-detects Rust/Python, dispatches to cargo/pytest/pip
- **`horus run`** for `.rs` and `.py` files — seamless polyglot execution
- **`horus test`** — runs cargo test AND pytest in mixed projects
- **`horus add`** — handles Rust (crates.io, git, path, registry) and Python (PyPI) deps
- **Feature flag system** (`features.rs`) — drivers + enable capabilities, dedup, passed to cargo
- **Error hint system** (`error_wrapper.rs`) — pattern-matches cargo/pip errors, suggests fixes
- **Dependency filtering** (`manifest.rs`) — crates_io_deps(), pypi_deps(), registry_deps(), path_deps()
- **`horus deps`** — tree, why, outdated, audit all implemented
- **`horus deploy`** — cross-compilation with arch detection, file transfer
- **`horus fmt/lint/doc/bench`** — dispatches to correct tool per language
- **Python PYTHONPATH** — auto-set for imports in run and test
- **`PackageInfo.edition`** field — exists in manifest.rs (just not used by cargo_gen)
- **Registry integration** — source resolution, package download, caching
- **Lockfile v3** — unified format across sources

---

## Workflow 1: `horus new` — Perfect First Run

**Current state:** Creates project, but IDE doesn't work. Student must manually configure rust-analyzer.

**Target state:** `horus new my_robot` creates a project that works immediately — build, run, IDE, everything.

### What `horus new` must generate:

```
my_robot/
├── horus.toml              # Source of truth
├── src/
│   └── main.rs             # (or main.py for --python)
└── .vscode/
    └── settings.json        # rust-analyzer config (auto-generated)
```

`.vscode/settings.json` (only if `.horus/Cargo.toml` will be used):
```json
{
  "rust-analyzer.linkedProjects": [".horus/Cargo.toml"]
}
```

### Changes needed:
- `commands/new.rs` — add `.vscode/settings.json` generation
- For Python projects: generate `pyrightconfig.json` or equivalent if needed
- Run `horus build` silently after `horus new` so `.horus/Cargo.toml` exists for IDE

---

## Workflow 2: `horus build` — Any Language, Clean Output

**Current state:** Works, but output shows `.horus/target/release/foo`. Errors show `.horus/src/main.rs:42`.

**Target state:** User sees `src/main.rs:42` in errors, `Built: my_robot` in success. Zero `.horus/` leaks.

### 2a. Path rewriting utility

```rust
// output.rs (new module)
pub fn rewrite_horus_paths(output: &str) -> String {
    output
        .replace(".horus/src/", "src/")
        .replace(".horus/target/", "target/")
        .replace(".horus/Cargo.toml", "horus.toml (generated)")
        .replace(".horus/pyproject.toml", "horus.toml (generated)")
        .replace(".horus/Cargo.lock", "horus.lock")
        .replace(" in .horus/", " ")
}
```

Apply at every error output site (~8 call sites in run_rust.rs, test.rs, install.rs, dispatch.rs, run/mod.rs).

### 2b. Success message cleanup

| Current | Target |
|---------|--------|
| `"Built: .horus/target/release/foo"` | `"Built: foo"` |
| `"No .horus/Cargo.toml found"` | `"Run 'horus build' first"` |
| `"cargo test in .horus/"` | `"Running tests..."` |
| `"Created .horus/ directory"` | `"Initialized build cache"` |
| `".horus/ uses 1.2GB"` | `"Build cache uses 1.2GB"` |

### 2c. Multi-language build

`horus build` must detect project languages and build all:

```
horus build
├─ Rust detected? → cargo build (via .horus/Cargo.toml)
├─ Python detected? → pip install deps (via .horus/pyproject.toml)
└─ C++ detected? → cmake build (future)
```

This already works. Just clean up the output.

---

## Workflow 3: `horus run` — Seamless Polyglot Execution

**Current state:** Works for .rs and .py. This is horus's superpower.

**Target state:** Same, but with clean output and full flag support.

```bash
horus run                     # runs default entry point (any language)
horus run main.rs             # Rust
horus run detector.py         # Python (with PYTHONPATH auto-set)
horus run sim                 # runs [scripts].sim from horus.toml
horus run --release           # optimized build
horus run --example teleop    # NEW: run example
horus run --features headless # NEW: feature override
```

### Changes needed:
- Add `--example <name>` flag → pass `--example` to cargo
- Add `--features <list>` flag → pass `--features` to cargo
- Add `--no-default-features` flag → pass through
- Clean output paths (Phase 2a/2b above)

---

## Workflow 4: `horus test` — Unified Test Runner

**Current state:** Runs cargo test OR pytest. Missing test filter flags.

**Target state:** Runs ALL tests across ALL languages. Full filter support.

```bash
horus test                    # runs Rust tests + Python tests
horus test my_func            # filter by name (both languages)
horus test --lib              # NEW: Rust library tests only
horus test --doc              # NEW: Rust doc tests only
horus test --no-fail-fast     # NEW: don't stop on first failure
horus test --features gpu     # NEW: test with feature flags
horus test --python           # Python tests only
horus test --rust             # Rust tests only
horus test --integration      # already works
```

### Changes needed:
- `test.rs` — add `--lib`, `--doc`, `--no-fail-fast`, `--features` flags
- Pass these to `cargo test` when running Rust tests
- Add `--python` / `--rust` language filters
- Apply path rewriting to test output

---

## Workflow 5: `horus add` — Universal Dependency Manager

**Current state:** Works for Rust and Python deps. This is the polyglot advantage.

**Target state:** Same, but smarter auto-detection and full feature support.

```bash
horus add serde                          # auto-detects Rust (crates.io)
horus add serde --features derive        # already works
horus add numpy                          # auto-detects Python (PyPI)
horus add opencv --lang system           # system library
horus add torch --lang python            # explicit Python
horus add my-driver --source horus       # from horus registry
horus add criterion --dev                # dev dependency
```

### Already works well. Minor improvements:
- Better auto-detection: if package exists on both crates.io and PyPI, prompt or use project's primary language
- `horus add --lang python` as explicit override

---

## Workflow 6: `horus deploy` — Build + Ship

**Current state:** Cross-compiles and transfers. deploy.yaml lives in `.horus/`.

**Target state:** deploy config in horus.toml `[deploy]` section or `deploy.yaml` at project root.

```toml
# horus.toml
[deploy]
target = "pi@192.168.1.50"
arch = "aarch64"
run = true
```

```bash
horus deploy                             # uses [deploy] from horus.toml
horus deploy pi@robot --arch aarch64     # override from CLI
```

### Changes needed:
- Move deploy.yaml lookup from `.horus/deploy.yaml` to project root `deploy.yaml`
- Support `[deploy]` section in horus.toml as alternative
- Auto-migrate `.horus/deploy.yaml` → `deploy.yaml` on first run

---

## Workflow 7: Generation Layer — Airtight Translation

**This is the core engine.** `cargo_gen.rs` must faithfully translate horus.toml to Cargo.toml without losing anything.

### Currently supported in cargo_gen.rs:
- ✅ `[package]` name, version
- ✅ `[dependencies]` with version, features, git, path
- ✅ `[dev-dependencies]`
- ✅ `[[bin]]` auto-detection from source files
- ✅ `[workspace]` isolation (empty, prevents parent inheritance)

### Must add to cargo_gen.rs:

| horus.toml | Generated Cargo.toml | Effort |
|------------|---------------------|--------|
| `edition = "2024"` | `edition = "2024"` | Tiny — one line change from hardcoded `"2021"` |
| `[profile.release]` | `[profile.release]` | Small — pass-through section |
| `[profile.dev]` | `[profile.dev]` | Small — same pattern |
| `[features]` | `[features]` | Small — pass-through section |
| `[lib]` type/name | `[lib]` crate-type/name | Small — one section |
| `[[example]]` | `[[example]]` | Small — one section |
| `[build]` script | `build = "../build.rs"` | Small — detect and point to root |

### Must add to manifest.rs:

Each field above needs a corresponding struct field in `HorusManifest` and TOML deserialization.

### Must add for .cargo/config.toml generation:

If horus.toml has `[build]` section with `rustflags` or `linker`:
```toml
# horus.toml
[build]
rustflags = ["-C", "link-arg=-fuse-ld=mold"]
linker = "aarch64-linux-gnu-gcc"
target = "aarch64-unknown-linux-gnu"
```
Generate `.horus/.cargo/config.toml`:
```toml
[build]
rustflags = ["-C", "link-arg=-fuse-ld=mold"]

[target.aarch64-unknown-linux-gnu]
linker = "aarch64-linux-gnu-gcc"
```

### Must add to pyproject_gen.rs:

If horus.toml has `[tool.pytest]`, `[tool.ruff]`, `[tool.mypy]` → forward to generated pyproject.toml.

---

## Workflow 8: Conflict Detection

**If user has BOTH horus.toml AND root Cargo.toml with dependencies:**

```rust
if root_cargo_has_deps && horus_toml_has_rust_deps {
    bail!(
        "Both horus.toml and Cargo.toml define Rust dependencies.\n\
         horus.toml is the source of truth. To consolidate:\n\
         \n\
         Run 'horus migrate' to move Cargo.toml deps into horus.toml."
    );
}
```

Clear error. One source of truth. No silent divergence.

---

## Workflow 9: Help Text and Docs — Zero `.horus/` Mentions

Grep all `--help` text and replace `.horus/` references:

| Current help text | New help text |
|---|---|
| `"default: .horus/blackbox/"` | `"default: project blackbox directory"` |
| `"Install to ~/.horus/cache/"` | `"Install to global cache"` |
| `"Path: .horus/config/params.yaml"` | `"Path: params.yaml"` |

---

## CLI Cleanup: 72 → ~35 Commands

Current CLI has 72 commands. Half are redundant, not CLI material, or not robotics workflow. Robotics developers use ~16 commands daily. The rest is bloat.

### Why runtime CLI commands are NOT bloat

In robotics deployment, the robot is headless in the field. You SSH in over a flaky network. CLI is all you have. Every runtime introspection command (`topic echo`, `param set`, `node kill`, `frame tune`, `action send-goal`, `topic pub cmd_vel`) is a real "SSH'd into a robot at 2am" workflow. These stay.

### Consolidate (merge into existing commands)

| Current | Merge into | Reason |
|---------|-----------|--------|
| `horus scripts <name>` | `horus run <name>` | `run` already resolves files; make it also resolve `[scripts]` entries |
| `horus cache info/clean/purge/list` | `horus clean --cache` / `horus clean --all` | Two cleaning systems → one with flags |
| `horus publish/unpublish` | `horus registry publish` | Grouped under registry |
| `horus enable/disable/verify` | `horus plugin enable/disable/verify` | Grouped under plugin |

### Remove (10 commands — duplicates and internal tools)

| Remove | Duplicate of | Reason |
|--------|-------------|--------|
| `horus install` | `horus add` | Same operation, two names. `add` is the canonical verb |
| `horus list` (packages) | `horus deps tree` | Both answer "what's in my project?" |
| `horus update` (packages) | `horus deps update` | Package updates belong under `deps` |
| `horus info` (package) | `horus search <pkg>` | Both show package details |
| `horus check` | `horus build --check` | Same operation, one less top-level command |
| `horus param dump` | `horus param save` | `save` without path → stdout, with path → file |
| `horus frame list` | `horus frame tree` | Tree shows everything list shows, plus hierarchy |
| `horus msg hash` | — | Internal wire-compatibility tool, not user-facing |
| `horus freeze` (top-level) | `horus env freeze` | Exact duplicate |
| `horus keygen` (top-level) | `horus auth generate-key` | Exact duplicate |

### Kept (justified — robot terminal + deployment use cases)

At the robot's physical terminal or over SSH, CLI IS the interface:

| Command | Use case |
|---------|----------|
| `horus config get/set` | No text editor on small screen |
| `horus frame calibrate` | On-site calibration with checkerboard |
| `horus frame hand-eye` | On-site hand-eye calibration |
| `horus frame tune` | Adjust sensor offset on live robot |
| `horus frame record/play/diff` | Debug transform issues without restarting |
| `horus record export` | Export to USB drive for lab analysis |
| `horus record inject` | Replay modified data for on-site testing |
| `horus record diff` | Regression testing after firmware update |
| `horus param reset` | Reset to defaults before demo |
| `horus action send-goal/cancel-goal` | Test navigation from terminal |
| `horus node pause/resume/restart` | Debug live robot without killing system |
| `horus upgrade` | Standard self-update expectation |
| `horus shell` | CI scripts / Docker entrypoints |

### Target CLI: ~63 commands (from 72)

10 removals (7 duplicates + 2 top-level duplicates + 1 internal) + 4 consolidations. Every remaining command is unique — no two commands do the same thing.

### Smart `horus init` (replaces `horus migrate`)

Current `horus init` is dumb — just creates blank horus.toml + .horus/ directory. Make it context-aware:

```
horus init (in empty directory)
  → creates blank horus.toml, .horus/, done

horus init (directory has Cargo.toml)
  → reads Cargo.toml [dependencies], [dev-dependencies], edition, profile
  → imports into horus.toml
  → prints: "Imported 12 dependencies from Cargo.toml"
  → asks: "Remove root Cargo.toml? horus.toml is now the source of truth [Y/n]"

horus init (directory has pyproject.toml)
  → reads [project.dependencies]
  → imports into horus.toml with lang = "python"
  → prints: "Imported 5 Python dependencies from pyproject.toml"

horus init (directory has CMakeLists.txt)
  → detects C++ project
  → creates horus.toml with language = "cpp"
  → scans find_package() calls → imports as C++ deps

horus init (directory has .rs files but no Cargo.toml)
  → creates horus.toml with language = "rust"
  → scans use statements → suggests common deps

horus init (directory has .py files)
  → creates horus.toml with language = "python"
  → scans imports → suggests deps (numpy, torch, etc.)

horus init (mixed — has both .rs and .py files)
  → creates horus.toml with language = "mixed"
  → imports from both Cargo.toml and pyproject.toml if present
```

**Files to change:** `commands/init.rs` — rewrite from 10-line stub to smart detection
**Eliminates:** `horus migrate` (never needs to exist)

### Smart auto-detection (reduce CLI flags and commands)

| Manual today | Make smart | Eliminated flag/command |
|---|---|---|
| `horus add numpy --lang python` | numpy only on PyPI → auto-detect | `--lang` flag rarely needed |
| `horus add serde --source crates.io` | serde only on crates.io → auto-detect | `--source` flag rarely needed |
| `horus test --rust` / `--python` | Run ALL detected languages automatically | `--rust`/`--python` flags (already works) |
| `--drivers camera` on build/run | Read from `[drivers]` in horus.toml | `--drivers` flag (already works) |
| `horus new --rust` / `--python` | Prompt interactively if not specified | Flags still available, but not required |
| `horus build --release` then `horus deploy` | `deploy` always builds release (why deploy debug?) | `--release` on deploy is implicit |
| `horus init` + `horus new` | `horus new` in existing dir = init + scaffolding. `horus init` in existing project = import | Two commands handle all cases |
| `horus build` then `horus run` | `horus run` already builds if needed | `build` before `run` is unnecessary (already works) |
| `horus deps tree` vs `horus deps list` | `horus deps` with no subcommand shows tree | Simpler default |

### Commands made smarter (not removed, but need fewer flags)

| Command | Current | Smart version |
|---------|---------|---------------|
| `horus add` | Needs `--lang` for ambiguous packages | Auto-detect from package registry. Only prompt if genuinely ambiguous (exists on both crates.io and PyPI) |
| `horus deploy` | Needs `--release` explicitly | Always release. `--debug` flag for the rare case you want debug deploy |
| `horus test` | Needs `--integration` for integration tests | Auto-run if `tests/` directory has integration-marked tests. `--unit-only` to skip |
| `horus clean` | Separate `horus cache` command | `horus clean` cleans build. `horus clean --all` includes package cache |
| `horus deps` | Must specify subcommand | `horus deps` alone = `horus deps tree` (most common use) |

---

## Implementation Plan

### Phase 1: Airtight Output (blocks ALL users)

| Task | File | Effort |
|------|------|--------|
| Create `rewrite_horus_paths()` | `output.rs` (new) | Small |
| Apply to cargo build stderr | `run_rust.rs` (3 sites) | Small |
| Apply to cargo test output | `test.rs` (1 site) | Small |
| Apply to pip errors | `install.rs` (1 site) | Small |
| Apply to dispatch (fmt/lint/doc) | `dispatch.rs` (1 site) | Small |
| Apply to multi-process prefixed output | `run/mod.rs` (1 site) | Small |
| Clean success messages (7 strings) | various | Small |
| Clean help text (5 strings) | `main.rs` | Small |

**Estimated: 1 day. Fixes the most visible DX problems.**

### Phase 2: Perfect First Run

| Task | File | Effort |
|------|------|--------|
| Generate `.vscode/settings.json` | `commands/new.rs` | Small |
| Silent `horus build` after `horus new` | `commands/new.rs` | Small |
| Move deploy.yaml to project root | `commands/deploy.rs` | Small |
| Move params.yaml to project root | `main.rs` | Small |

**Estimated: half day. Fixes first-run experience.**

### Phase 3: Complete Generation Layer

| Task | File | Effort |
|------|------|--------|
| `edition` field (replace hardcode) | `manifest.rs` + `cargo_gen.rs` | Tiny |
| `[profile.*]` pass-through | `manifest.rs` + `cargo_gen.rs` | Small |
| `[features]` pass-through | `manifest.rs` + `cargo_gen.rs` | Small |
| `[lib]` section | `manifest.rs` + `cargo_gen.rs` | Small |
| `[[example]]` section | `manifest.rs` + `cargo_gen.rs` | Small |
| `[build]` script detection | `manifest.rs` + `cargo_gen.rs` | Small |
| `.cargo/config.toml` for rustflags/linker | `cargo_gen.rs` | Medium |
| Python tool config forwarding | `manifest.rs` + `pyproject_gen.rs` | Small |

**Estimated: 2-3 days. Makes horus.toml feature-complete.**

### Phase 4: CLI Completeness

| Task | File | Effort |
|------|------|--------|
| `--features` on build/run/test | `main.rs` + `run_rust.rs` + `test.rs` | Medium |
| `--lib`/`--doc`/`--no-fail-fast` on test | `main.rs` + `test.rs` | Small |
| `--example` on run | `main.rs` + `run_rust.rs` | Small |
| `horus deps update` | `deps.rs` | Small |
| Conflict detection (both files have deps) | `dispatch.rs` or `workspace.rs` | Small |

**Estimated: 1-2 days. Closes remaining CLI gaps.**

### Phase 5: Smart Init & Auto-Detection

| Task | File | Effort |
|------|------|--------|
| Smart `horus init` — detect Cargo.toml, import deps | `commands/init.rs` | Medium |
| Smart `horus init` — detect pyproject.toml, import deps | `commands/init.rs` | Small |
| Smart `horus init` — detect .rs/.py/.cpp files, set language | `commands/init.rs` | Small |
| `horus add` — auto-detect language from package registry | `commands/pkg.rs` | Medium |
| `horus deploy` — default to release (remove need for `--release`) | `commands/deploy.rs` | Tiny |
| `horus deps` — no subcommand defaults to `tree` | `commands/deps.rs` | Tiny |

**Estimated: 1-2 days. Eliminates `horus migrate` and reduces flag usage.**

### Phase 6: CLI Dedup (72 → ~63 commands)

| Task | File | Effort |
|------|------|--------|
| Merge `horus scripts` into `horus run` (resolve [scripts] entries) | `commands/run/mod.rs` | Small |
| Merge `horus cache` into `horus clean --cache` | `main.rs` + `commands/clean.rs` | Small |
| Remove `horus install` (duplicate of `horus add`) | `main.rs` | Small |
| Remove `horus list` (duplicate of `horus deps tree`) | `main.rs` | Small |
| Remove `horus update` (duplicate of `horus deps update`) | `main.rs` | Small |
| Remove `horus info` (duplicate of `horus search`) | `main.rs` | Small |
| Remove `horus check` (add `--check` flag to `horus build`) | `main.rs` + `run_rust.rs` | Small |
| Remove `horus param dump` (merge into `horus param save`) | `main.rs` | Tiny |
| Remove `horus frame list` (duplicate of `horus frame tree`) | `main.rs` | Tiny |
| Remove `horus msg hash` | `main.rs` | Tiny |
| Remove top-level `horus freeze` (keep `horus env freeze`) | `main.rs` | Tiny |
| Remove top-level `horus keygen` (keep `horus auth generate-key`) | `main.rs` | Tiny |
| Group `horus publish/unpublish` under `horus registry` | `main.rs` | Small |
| Group `horus enable/disable/verify` under `horus plugin` | `main.rs` | Small |

**Estimated: 1 day. Removes all duplicates. Zero functionality lost — every removed command has an exact equivalent.**

### Phase 7: Polish

| Task | File | Effort |
|------|------|--------|
| Binary symlink after build | `run_rust.rs` | Small |
| `[workspace]` members support | `manifest.rs` + `cargo_gen.rs` | Medium |
| Better language auto-detection for `horus add` | `pkg.rs` | Small |

**Estimated: 1-2 days. Nice-to-have refinements.**

---

## Total Effort: ~10-12 days

After this, a developer's entire interaction with horus is:

```bash
horus new my_robot              # perfect project, IDE works
horus add serde --features derive  # any language, any source
horus add numpy                 # auto-detects Python
horus build --release           # all languages, clean output
horus test                      # cargo test + pytest, unified
horus run                       # .rs or .py, seamless
horus deploy                    # cross-compile and ship
horus monitor -t                # runtime dashboard
```

One tool. One file. Any language. Zero `.horus/` exposure. Cargo-grade polish.

---

## Success Criteria

### Output
- [ ] `grep -rn '\.horus' horus_manager/src/ | grep -E 'println|eprintln|format!|bail!'` → zero hits
- [ ] Compiler errors show `src/main.rs`, never `.horus/src/main.rs`

### First run
- [ ] `horus new my_robot && cd my_robot && code .` → rust-analyzer works immediately

### Generation layer
- [ ] horus.toml supports: edition, profiles, features, lib, examples, build scripts
- [ ] Both Cargo.toml + horus.toml with deps → clear error, not silent divergence

### CLI completeness
- [ ] `horus build --features headless --release` works
- [ ] `horus test --lib --no-fail-fast` works
- [ ] `horus run --example teleop` works
- [ ] `horus run sim` resolves from `[scripts]` (no separate `horus scripts` command)
- [ ] `horus deps update` works
- [ ] Mixed Rust+Python project: `horus test` runs both, `horus add` handles both
- [ ] No workflow requires `cd .horus && cargo ...`

### CLI cleanup
- [ ] No duplicate top-level commands (freeze/keygen removed, install=alias for add)
- [ ] `horus run sim` resolves from `[scripts]` (scripts command merged into run)
- [ ] `horus clean --cache` replaces 4 separate cache subcommands
- [ ] All runtime + on-robot commands remain — robot terminal is the primary interface
- [ ] Only `horus msg hash` removed (internal tool, not user-facing)
