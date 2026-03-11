# Unified Manifest Redesign: horus.toml as Single Source of Truth

## Architecture Principle: Lightweight Core, Heavy Plugins

horus core is a **lightweight orchestrator**. It has zero domain knowledge — no physics engine, no device drivers, no ML frameworks, no rendering. It dispatches to subprocesses (cargo, python, ruff, pytest) and plugins (horus-sim, horus-rplidar, horus-bag). Every new core command must add **zero new Cargo dependencies** — if it needs heavy functionality, it spawns an external tool or delegates to a plugin.

The plugin system exists for this boundary. Plugins are standalone binaries that communicate with horus via shared-memory topics (file-mapped I/O) and the filesystem. They carry their own dependencies (Bevy, Rapier3D, librealsense, etc.) and are opt-in. Users install only what they need.

```
horus core (always installed, lightweight)
  ├── project: new, init, run, build, test, check, clean, launch
  ├── ipc: topic, node, param, service, action, frame, msg
  ├── debug: log, blackbox, monitor*, record
  ├── packages: install, remove, search, update, info
  ├── plugins: enable, disable, verify
  ├── dx: fmt, lint, doc, bench, doctor, upgrade, migrate, shell, deps, config
  └── deploy: publish, unpublish, deploy, env, auth, keygen, completion
  * monitor is feature-gated (default on, excluded with --no-default-features)

plugins (opt-in, heavy, own dependencies)
  ├── horus-sim       → 3D simulation (Bevy, Rapier3D)
  ├── horus-hw        → hardware detection meta-plugin
  ├── horus-rplidar   → LiDAR driver (serial, USB)
  ├── horus-realsense → depth camera (librealsense2)
  ├── horus-dynamixel → servo driver (serial protocol)
  ├── horus-bno055    → IMU driver (I2C)
  ├── horus-terra     → hardware abstraction (26 terra crates)
  ├── horus-plot      → live topic visualization (TUI charts)
  ├── horus-bag       → data recording/replay
  ├── horus-ros-bridge→ ROS2 interop (rclrs, DDS)
  ├── horus-gazebo    → Gazebo sim integration
  └── community       → anything users publish to registry
```

### Command Classification: Core vs Plugin vs Redesign

**Core — keep as-is (33 commands, 80+ subcommands, no changes needed):**

| Category | Commands |
|----------|----------|
| Project | `init`, `new`, `run`, `build`, `launch` |
| IPC | `topic` (list/echo/info/hz/pub/bw), `node` (list/info/kill/restart/pause/resume) |
| IPC | `param` (list/get/set/delete/reset/load/save/dump) |
| IPC | `frame` (list/echo/tree/info/can/hz/record/play/diff/tune/calibrate/handeye) |
| IPC | `service` (list/call/info/find), `action` (list/info/send-goal/cancel-goal) |
| IPC | `msg` (list/info/hash), `discover` |
| Debug | `log`, `blackbox`, `record` (list/info/delete/replay/diff/export/inject) |
| Debug | `cache` (info/clean/purge/list) |
| Packages | `install`, `remove`, `search`, `list`, `update`, `info` |
| Plugins | `enable`, `disable`, `verify` |
| Publish | `publish`, `unpublish`, `keygen` |
| Deploy | `deploy`, `env` (list/info/freeze/restore), `auth` (login/logout/whoami/generate-key/keys) |
| System | `completion` |

**Core — redesign/refactor (11 commands):**

| Command | What changes |
|---------|-------------|
| `test` | Add Python pytest dispatch (auto-detect language, run both) |
| `check` | Fast default (manifest + compile). `--full` for lint/fmt/deps/security |
| `clean` | Only clean horus-owned state. Add `--cache` flag. No `--deep` |
| `new` | Stop generating Cargo.toml/pyproject.toml. Add `[dependencies]` to horus.toml |
| `build` | Always generate `.horus/Cargo.toml` from horus.toml. One code path |
| `run` | Same as build + add `[scripts]` resolution from horus.toml |
| `install` | Write to horus.toml `[dependencies]` instead of `cargo add`/`pip install` |
| `remove` | Edit horus.toml instead of `cargo remove` |
| `env freeze` | Read from horus.toml + generate unified `horus.lock` |
| `env restore` | Write horus.toml from lock, then rebuild |
| `monitor` | Feature-gate heavy deps (axum, ratatui, qrcode, argon2) behind `monitor` feature |

**Core — add new (10 commands, 0 new Cargo dependencies):**

| Command | What it does | How (no new deps) |
|---------|-------------|-------------------|
| `fmt` | Format all code | Spawns `cargo fmt` / `ruff format` |
| `lint` | Lint all code | Spawns `cargo clippy` / `ruff check` |
| `doc` | Generate docs | Spawns `cargo doc` / `pdoc` |
| `bench` | Run benchmarks | Spawns `cargo bench` / `pytest-benchmark` |
| `doctor` | Ecosystem health check | File checks + subprocess `--version` probes |
| `upgrade` | Upgrade horus ecosystem | HTTP (reqwest exists) + subprocess builds |
| `migrate` | Convert existing project | Parses toml (toml_edit exists) |
| `shell` | Activated subshell | Spawns `$SHELL` with activation script |
| `deps` | Dep insight (tree/why/outdated/audit) | Spawns `cargo tree` / `pipdeptree` |
| `config` | Edit horus.toml from CLI | Reads/writes toml (toml_edit exists) |

**Plugins — opt-in, installed via `horus install` (11 plugins):**

| Plugin package | Command | Heavy deps (not in horus core) |
|---------------|---------|-------------------------------|
| `horus-sim` | `horus sim start/stop/list/reset/step` | Bevy 0.15, Rapier3D 0.22 |
| `horus-hw` | `horus hw detect/drivers/info` | Device scanning, terra-core |
| `horus-rplidar` | `horus hw rplidar start/stop/config` | Serial protocol, USB |
| `horus-realsense` | `horus hw realsense start/config` | librealsense2 native binding |
| `horus-dynamixel` | `horus hw dynamixel scan/config` | Dynamixel serial protocol |
| `horus-bno055` | `horus hw bno055 start/calibrate` | I2C driver, sensor fusion |
| `horus-terra` | `horus terra start/status` | 26-crate HAL (terra workspace) |
| `horus-plot` | `horus plot topic.field --window 100` | TUI charting, sparklines |
| `horus-bag` | `horus bag record/play/info/export/filter` | Data format, compression |
| `horus-ros-bridge` | `horus ros bridge/topics/services` | rclrs, DDS, ROS2 runtime |
| `horus-gazebo` | `horus gazebo start/spawn/world` | Gazebo transport, protobuf |

**Feature-gated (default on, opt-out for minimal installs):**

| Feature | Commands | Deps gated | Exclude with |
|---------|----------|-----------|-------------|
| `monitor` | `horus monitor`, `horus monitor --tui` | axum, tower, tower-http, ratatui, crossterm, qrcode, argon2, rpassword | `--no-default-features` |
| `mdns` | `horus discover` | mdns (already gated) | omit `--features mdns` |

**Summary:**

| Category | Count |
|----------|-------|
| Core — keep as-is | 33 commands (80+ subcommands) |
| Core — redesign/refactor | 11 commands |
| Core — add new | 10 commands |
| Plugins | 11 plugins |
| Feature-gated | 2 (monitor, mdns) |
| **Total core commands** | **54** |

## Goal

Users only interact with `horus.toml` and their source files. No `Cargo.toml`, no `pyproject.toml` at root. All build artifacts live in `.horus/`. The horus CLI is the only tool needed.

## Project Structure

### Before (current)
```
my-project/
├── horus.toml          # config only (no deps)
├── Cargo.toml          # Rust deps (user-managed)
├── pyproject.toml      # Python deps (user-managed)
├── src/main.rs
└── .horus/
    ├── Cargo.toml      # generated for standalone files (duplicate!)
    ├── target/
    └── packages/
```

### After (unified)
```
my-project/
├── horus.toml          # THE manifest (config + deps + everything)
├── main.rs             # user code (or main.py, or both)
└── .horus/             # ALL generated (gitignored)
    ├── Cargo.toml      # always generated from horus.toml
    ├── pyproject.toml   # always generated from horus.toml
    ├── Cargo.lock       # generated, committed for reproducibility
    ├── target/          # cargo build output
    └── packages/        # installed horus registry packages + tracking
```

## horus.toml Format

```toml
[package]
name = "my-robot"
version = "1.0.0"
description = "Navigation robot with LIDAR"
authors = ["dev@softmata.com"]
license = "Apache-2.0"
edition = "1"
package-type = "node"
categories = ["robotics", "navigation"]

[drivers]
camera = "opencv"
lidar = true

[ignore]
files = ["test_*.rs"]
directories = ["data/"]

[dependencies]
# Horus registry packages (default source)
horus_library = "0.1.9"
my-horus-sensor = "^2.0"

# Rust crates from crates.io
serde = { version = "1.0", features = ["derive"], source = "crates.io" }
tokio = { version = "1", features = ["full"], source = "crates.io" }
rapier3d = { version = "0.22", source = "crates.io" }
nalgebra = { version = "0.33", source = "crates.io" }

# Python packages from PyPI
numpy = { version = ">=1.24", source = "pypi" }
opencv-python = { version = ">=4.8", source = "pypi" }

# System packages
libudev-dev = { source = "system" }

# Path dependencies (local development)
my-local-lib = { path = "../my-local-lib" }

# Git dependencies
some-fork = { git = "https://github.com/org/repo", branch = "main" }

[dependencies.dev]
# Dev-only deps (not included in publish/freeze)
criterion = { version = "0.5", source = "crates.io" }
pytest = { version = ">=7.0", source = "pypi" }

[enable]
features = ["cuda", "python"]
```

### Dependency Source Detection

When no `source` is specified:
1. Check horus registry first
2. Auto-detect from crates.io / PyPI (existing `detect_package_source()` logic)
3. If ambiguous, prompt user (existing behavior)

Simple string values (e.g., `horus_library = "0.1.9"`) default to horus registry — same as today.

## CLI Commands — What Changes

### `horus new`

**Before:** Creates `horus.toml` + `Cargo.toml` + `src/main.rs` (Rust) or `horus.toml` + `pyproject.toml` + `main.py` (Python)

**After:** Creates `horus.toml` + `main.rs` (Rust) or `horus.toml` + `main.py` (Python). No native build files.

```bash
horus new my-robot --rust
# Creates:
#   my-robot/horus.toml    (with [dependencies] including horus_library)
#   my-robot/main.rs

horus new my-robot --python
# Creates:
#   my-robot/horus.toml    (with [dependencies] including horus-robotics from pypi)
#   my-robot/main.py
```

**Files to change:** `commands/new.rs`
- Remove `Cargo.toml` generation (lines 250-302)
- Remove `pyproject.toml` generation (lines 305-332)
- Add `[dependencies]` section to generated `horus.toml`
- Generate `main.rs` directly (not inside `src/`)

### `horus build`

**Before:** Two paths — uses root `Cargo.toml` if exists, generates `.horus/Cargo.toml` for standalone files.

**After:** One path — always generates `.horus/Cargo.toml` from `horus.toml`, always builds from `.horus/`.

```bash
horus build
# 1. Read horus.toml [dependencies]
# 2. Generate .horus/Cargo.toml (Rust deps) and/or .horus/pyproject.toml (Python deps)
# 3. Run cargo build from .horus/ (or skip for Python)
# 4. Output binary to .horus/target/{debug,release}/
```

**Files to change:** `commands/run/run_rust.rs`
- Remove the `if Path::new(CARGO_TOML).exists()` branch in `execute_build_only()` (line 97)
- Remove the `if Path::new(CARGO_TOML).exists()` branch in `execute_with_scheduler()` (line 702)
- Always generate `.horus/Cargo.toml` from `horus.toml`
- The generation logic already exists (lines 136-218), just needs to read deps from `horus.toml` instead of hardcoding horus path deps only

### `horus run`

**Before:** Two paths for Rust, detects language from file extension.

**After:** One path — always generates from `horus.toml`, always builds from `.horus/`.

```bash
horus run                    # auto-detect main.rs or main.py
horus run main.rs            # explicit Rust file
horus run main.py            # explicit Python file
horus run node1.rs node2.py  # mixed concurrent execution
```

**Files to change:** `commands/run/run_rust.rs`, `commands/run/mod.rs`
- `execute_with_scheduler()`: Remove root `Cargo.toml` branch, always use generated `.horus/Cargo.toml`
- `build_rust_files_batch()`: Already generates `.horus/Cargo.toml` — just read deps from `horus.toml`
- `build_file_for_concurrent_execution()`: Same change
- `execute_from_cargo_toml()`: Remove entirely (no more root Cargo.toml to execute from)

### `horus test`

**Before:** Runs `cargo test` in `.horus/` directory, checks for `.horus/Cargo.toml`.

**After:** Same behavior, but `.horus/Cargo.toml` is always generated from `horus.toml` before testing.

```bash
horus test                   # run all tests
horus test my_test           # filter by name
horus test --sim             # simulation mode
horus test --parallel        # multi-threaded
```

**Files to change:** `commands/test.rs`
- Before running tests, ensure `.horus/Cargo.toml` is generated from `horus.toml`
- Remove check for root `Cargo.toml` staleness comparison (line 31-50)

### `horus install`

**Before:** Detects source, installs to `.horus/packages/` or runs `cargo add`/`pip install`. Doesn't update `horus.toml`.

**After:** Detects source, adds to `horus.toml [dependencies]`, installs as needed.

```bash
horus install serde --features derive
# 1. detect_package_source("serde") → CratesIO
# 2. Add to horus.toml: serde = { version = "1.0", features = ["derive"], source = "crates.io" }
# 3. Regenerate .horus/Cargo.toml
# (no cargo add — horus.toml IS the manifest)

horus install numpy
# 1. detect_package_source("numpy") → PyPI
# 2. Add to horus.toml: numpy = { version = ">=1.24", source = "pypi" }
# 3. pip install to .horus/packages/ (for runtime PYTHONPATH)

horus install my-horus-pkg
# 1. detect_package_source("my-horus-pkg") → Registry
# 2. Add to horus.toml: my-horus-pkg = "^2.0"
# 3. Download to .horus/packages/
```

**Files to change:** `registry/install.rs`
- `install_to_target()`: After detecting source, write dep to `horus.toml [dependencies]` via manifest API
- `install_from_cratesio()`: Remove `cargo add` call — just write to `horus.toml`. `.horus/Cargo.toml` is regenerated on next build
- `install_from_pypi()`: Remove `pyproject.toml` update — just write to `horus.toml`. Still pip install to cache for PYTHONPATH
- Remove the lib/bin detection in `install_from_cratesio()` — all crates.io deps are library deps (tracked in horus.toml, resolved via generated Cargo.toml). Binary-only CLI tools use `horus install --tool`

### `horus remove`

**Before:** Removes from `.horus/packages/`, tracking jsons, optionally runs `cargo remove`.

**After:** Removes from `horus.toml [dependencies]`.

```bash
horus remove serde
# 1. Remove "serde" from horus.toml [dependencies]
# 2. Clean up .horus/packages/ tracking if present
# 3. .horus/Cargo.toml will be regenerated on next build
```

**Files to change:** `commands/pkg.rs`
- `run_remove()`: Read `horus.toml`, remove the dep entry, save. Remove tracking files. No more `cargo remove` calls.

### `horus env freeze`

**Before:** Scans `.horus/packages/` only, misses Cargo.lock and pyproject.toml deps.

**After:** Reads `horus.toml [dependencies]` (the single source of truth) + `.horus/Cargo.lock` for exact Rust pins.

```bash
horus env freeze
# 1. Read horus.toml [dependencies] — has ALL deps
# 2. Read .horus/Cargo.lock — has exact pinned Rust versions
# 3. Read pip show for exact Python versions
# 4. Combine into horus-freeze.toml
```

**Files to change:** `registry/publish.rs`
- `freeze()`: Read deps from `horus.toml` instead of scanning `.horus/packages/`. Augment with exact versions from `.horus/Cargo.lock` and pip.

### `horus env restore`

**Before:** Routes everything through `client.install()` or source-specific installers.

**After:** Writes `horus.toml`, then runs `horus build` to regenerate everything.

```bash
horus env restore horus-freeze.toml
# 1. Parse freeze manifest
# 2. Generate horus.toml [dependencies] from package list
# 3. Download horus registry packages to .horus/packages/
# 4. Run horus build (generates .horus/Cargo.toml, resolves all deps)
```

**Files to change:** `commands/env.rs`
- `restore_packages()`: Write deps to `horus.toml`, then trigger build to resolve.

### `horus check`

Full project validation — not just "does it compile?" but everything: manifest, types, deps, security, hardware, code quality.

```bash
horus check                  # fast check: manifest + compile only (<5s)
horus check --full           # all phases (lint, fmt, deps, security, hardware)
horus check --strict         # treat warnings as errors (for CI)
horus check --json           # machine-readable output
```

**Default (fast):** manifest validation + `cargo check` + `py_compile`. Runs in seconds.

**`--full` adds** (each already has its own command — `horus check --full` just runs them all):

| Phase | What | Own command |
|-------|------|-------------|
| Lint | Code quality | `horus lint` |
| Format | Style compliance | `horus fmt --check` |
| Deps | Lockfile freshness, missing deps | `horus deps outdated` |
| Security | Known vulnerabilities | `horus deps audit` |
| Hardware | Device access, user groups | `horus doctor --category hw` |

**Files to change:** `commands/check.rs`
- Point `cargo check` at `.horus/Cargo.toml` instead of root
- Add lint, format, type-check, security, deps phases
- Integrate with `dispatch.rs` for Python tool detection

### `horus clean`

Clean everything — not just build artifacts but the entire ecosystem state.

```bash
horus clean                  # build artifacts only (safe default)
horus clean --shm            # shared memory only
horus clean --cache          # package cache only
horus clean --all            # everything horus owns
horus clean --dry-run        # show what would be removed
horus clean --force          # remove active SHM namespaces too
horus clean --json           # machine-readable output
```

**What each flag cleans (only horus-owned state, never touches cargo/pip caches):**

| Scope | What | Location |
|-------|------|----------|
| (default) | Build artifacts | `.horus/target/`, `target/` |
| `--shm` | Stale SHM namespaces | `/dev/shm/horus_*` |
| `--cache` | Package cache | `~/.horus/cache/` |
| `--all` | All of the above + generated files | `.horus/Cargo.toml`, `.horus/pyproject.toml`, `horus.lock` |
| `--all` | Orphaned workspaces | stale entries in `~/.horus/workspaces.json` |
| `--all` | Stale recordings | `~/.horus/recordings/` older than 30 days |
| `--all` | Stale blackbox data | `~/.horus/blackbox/` older than 30 days |

**Size report before cleaning:**
```
horus clean --dry-run --all
  .horus/target/         2.1 GB
  target/                847 MB
  /dev/shm/horus_*       14 MB (2 stale namespaces)
  ~/.horus/cache/        312 MB (23 packages, 2 orphaned)
  .horus/Cargo.toml      4 KB (generated)
  ~/.horus/recordings/   89 MB (3 files > 30 days)
  ─────────────────────────────
  Total reclaimable:     3.3 GB
```

### `horus deploy`

**Before:** Runs `cargo build --release` at project root.

**After:** Generates `.horus/Cargo.toml`, runs `cargo build --release` from `.horus/`.

**Files to change:** `commands/deploy.rs`
- Build from `.horus/` like everything else.

### `horus publish`

**Before:** Reads root `Cargo.toml` for version/metadata, packages source files.

**After:** Reads `horus.toml` for everything (already mostly does this).

**Files to change:** `registry/publish.rs`
- Ensure all metadata comes from `horus.toml`, not `Cargo.toml`.

### `horus add` (alias)

`horus add` becomes an alias for `horus install`. Both write to `horus.toml`.

## .horus/Cargo.toml Generation

New module: **`src/cargo_gen.rs`** (or extend existing `run_rust.rs`)

```rust
/// Generate .horus/Cargo.toml from horus.toml manifest
pub fn generate_cargo_toml(manifest: &HorusManifest, entry_files: &[PathBuf]) -> Result<()>
```

### Generation Logic

1. Read `horus.toml` via `HorusManifest::load_from()`
2. Build `[package]` section from `manifest.package`
3. Build `[[bin]]` entries from source files (auto-detect or explicit)
4. Build `[dependencies]` from `manifest.dependencies`:
   - Horus registry deps → `path = ".horus/packages/{name}"` (if downloaded)
   - Crates.io deps → standard version string with features
   - Skip PyPI/system deps (not Cargo deps)
5. Add horus core path deps (horus, horus_core, horus_library, horus_macros) via `find_horus_source_dir()`
6. Write to `.horus/Cargo.toml`

### Generated Cargo.toml Example

```toml
# AUTO-GENERATED by horus from horus.toml — do not edit
[package]
name = "my-robot"
version = "1.0.0"
edition = "2021"

[workspace]

[[bin]]
name = "my-robot"
path = "../main.rs"

[dependencies]
# Horus core (path deps)
horus = { path = "/home/user/softmata/horus/horus" }
horus_core = { path = "/home/user/softmata/horus/horus_core" }
horus_library = { path = "/home/user/softmata/horus/horus_library", features = ["camera"] }
horus_macros = { path = "/home/user/softmata/horus/horus_macros" }

# From horus.toml [dependencies]
serde = { version = "1.0", features = ["derive"] }
tokio = { version = "1", features = ["full"] }
rapier3d = "0.22"

# Horus registry packages (as path deps)
my-horus-sensor = { path = "../.horus/packages/my-horus-sensor" }
```

## .horus/pyproject.toml Generation

For Python deps, generate `.horus/pyproject.toml`:

```toml
# AUTO-GENERATED by horus from horus.toml — do not edit
[project]
name = "my-robot"
version = "1.0.0"
requires-python = ">=3.10"
dependencies = [
    "numpy>=1.24",
    "opencv-python>=4.8",
    "horus-robotics",
]
```

This is used for `pip install -e .horus/` or for PYTHONPATH construction. Python deps are also installed to `.horus/packages/` for isolation.

## HorusManifest Changes

### Current struct (manifest.rs)

```rust
pub struct HorusManifest {
    pub package: PackageInfo,
    pub drivers: BTreeMap<String, DriverValue>,
    pub ignore: IgnoreConfig,
    pub enable: Vec<String>,
}
```

### New struct

```rust
pub struct HorusManifest {
    pub package: PackageInfo,
    pub drivers: BTreeMap<String, DriverValue>,
    pub ignore: IgnoreConfig,
    pub enable: Vec<String>,
    pub dependencies: BTreeMap<String, DependencyValue>,       // NEW
    pub dev_dependencies: BTreeMap<String, DependencyValue>,   // NEW (optional)
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum DependencyValue {
    /// Simple version string: `horus_library = "0.1.9"`
    Version(String),
    /// Detailed spec: `serde = { version = "1.0", features = ["derive"], source = "crates.io" }`
    Detailed(DetailedDependency),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetailedDependency {
    pub version: Option<String>,
    pub source: Option<String>,           // "crates.io", "pypi", "system", "git"
    pub features: Option<Vec<String>>,    // Cargo features
    pub path: Option<String>,             // Local path dep
    pub git: Option<String>,              // Git URL
    pub branch: Option<String>,           // Git branch
    pub tag: Option<String>,              // Git tag
    pub rev: Option<String>,              // Git rev
    pub optional: Option<bool>,           // Optional dependency
}
```

Note: The old `DependencyValue` and `DetailedDependency` types were deleted in the manifest separation. They need to be re-added with the new `source` field. The existing `dependency_resolver.rs` already has `DependencySource` and `DependencySpec` that map to these — reuse that logic.

## Lockfile Changes

### horus.lock — Delete

The v2 lockfile (just a config hash) serves no purpose when `horus.toml` is the single manifest and `.horus/Cargo.lock` handles Rust pinning.

**Files to change:** `lockfile.rs` — can be deleted or repurposed.

### .horus/Cargo.lock — Rust version pinning

Cargo generates this automatically. It should be committed to git for reproducibility (add to `.gitignore` exception or keep outside `.horus/` — TBD).

## Migration

### `horus migrate` (new command)

For existing projects with root `Cargo.toml` / `pyproject.toml`:

```bash
horus migrate
# 1. Read Cargo.toml [dependencies] → add to horus.toml [dependencies] with source = "crates.io"
# 2. Read pyproject.toml [project.dependencies] → add to horus.toml [dependencies] with source = "pypi"
# 3. Move Cargo.toml → .horus/Cargo.toml.bak (backup)
# 4. Move pyproject.toml → .horus/pyproject.toml.bak (backup)
# 5. Print summary
```

### Backward compatibility

- Old `horus.toml` without `[dependencies]` still works (empty deps)
- `horus run` with a root `Cargo.toml` and no `horus.toml` should warn and suggest migration

## Implementation Order

### Phase 1: Manifest (foundation)
1. Add `dependencies` and `dev_dependencies` fields to `HorusManifest` in `manifest.rs`
2. Add `DependencyValue` and `DetailedDependency` types
3. Ensure backward compat: old `horus.toml` without `[dependencies]` still loads

### Phase 2: Cargo.toml generation
4. Create `cargo_gen.rs` — generate `.horus/Cargo.toml` from `horus.toml`
5. Create `pyproject_gen.rs` — generate `.horus/pyproject.toml` from `horus.toml`

### Phase 3: Build/Run/Test
6. Modify `run_rust.rs` — remove root `Cargo.toml` path, always use generated `.horus/Cargo.toml`
7. Modify `run_rust.rs` — `execute_build_only()`, `execute_with_scheduler()`, `build_rust_files_batch()`, `build_file_for_concurrent_execution()` all call `cargo_gen::generate()` first
8. Modify `test.rs` — generate before test, run from `.horus/`
9. Remove `execute_from_cargo_toml()` — no longer needed

### Phase 4: Install/Remove
10. Modify `registry/install.rs` — `install_to_target()` writes to `horus.toml` instead of `Cargo.toml`/`pyproject.toml`
11. Remove `cargo add` / `pyproject.toml` modification from install functions
12. Modify `commands/pkg.rs` — `run_remove()` edits `horus.toml` instead of running `cargo remove`
13. Remove tracking JSON files (`.crates-io.json`, `.pypi.json`) — `horus.toml` is the tracking

### Phase 5: Freeze/Restore
14. Modify `registry/publish.rs` — `freeze()` reads `horus.toml` + `.horus/Cargo.lock`
15. Modify `commands/env.rs` — `restore_packages()` writes `horus.toml`, then rebuilds

### Phase 6: New/Check/Deploy/Publish
16. Modify `commands/new.rs` — stop generating `Cargo.toml`/`pyproject.toml`, add `[dependencies]` to `horus.toml`
17. Modify `commands/check.rs` — validate from `.horus/`
18. Modify `commands/deploy.rs` — build from `.horus/`
19. Modify `registry/publish.rs` — package from `horus.toml` metadata only

### Phase 7: Cleanup
20. Delete `horus.lock` / `lockfile.rs` (or repurpose for config hash only)
21. Add `horus migrate` command
22. Update `.gitignore` template to include `.horus/` (except `.horus/Cargo.lock`)
23. Remove the `add_dep_to_pyproject_toml` / `remove_dep_from_pyproject_toml` / tracking JSON helpers added in the previous partial refactor
24. Update example projects (`examples/quadruped/`, `examples/differential_drive/`) to new format

## Files Affected (complete list)

| File | Change Type | Description |
|------|------------|-------------|
| `manifest.rs` | MODIFY | Add `dependencies`, `DependencyValue`, `DetailedDependency` |
| `cargo_gen.rs` | NEW | Generate `.horus/Cargo.toml` from `horus.toml` |
| `pyproject_gen.rs` | NEW | Generate `.horus/pyproject.toml` from `horus.toml` |
| `commands/new.rs` | MODIFY | Stop generating Cargo.toml/pyproject.toml, add deps to horus.toml |
| `commands/run/run_rust.rs` | MODIFY | Remove root Cargo.toml branches, always use generated |
| `commands/run/mod.rs` | MODIFY | Remove `execute_from_cargo_toml` references |
| `commands/test.rs` | MODIFY | Generate before test, always from .horus/ |
| `commands/check.rs` | MODIFY | Check from .horus/ |
| `commands/deploy.rs` | MODIFY | Build from .horus/ |
| `commands/pkg.rs` | MODIFY | Install/remove writes to horus.toml |
| `commands/env.rs` | MODIFY | Freeze reads horus.toml, restore writes horus.toml |
| `registry/install.rs` | MODIFY | Write to horus.toml instead of Cargo.toml/pyproject.toml |
| `registry/publish.rs` | MODIFY | Freeze from horus.toml + .horus/Cargo.lock |
| `registry/helpers.rs` | MODIFY | Remove pyproject.toml/Cargo.toml direct modification helpers |
| `registry/mod.rs` | MODIFY | Update re-exports |
| `lockfile.rs` | DELETE or SIMPLIFY | No more horus.lock needed |
| `deps.rs` | MODIFY | Read deps from horus.toml instead of scanning imports |
| `commands/run/install.rs` | MODIFY | Simplify — deps come from horus.toml |

## cargo/pip at Root No Longer Works

There is no `Cargo.toml` or `pyproject.toml` at root. Running `cargo build` or `pip install` at root will fail. This is intentional — horus is the only CLI.

Every cargo/pip operation must have a horus equivalent:

| User wants to | cargo/pip | horus equivalent | Status |
|---|---|---|---|
| Add dep | `cargo add serde` | `horus install serde` | Exists |
| Remove dep | `cargo remove serde` | `horus remove serde` | Exists |
| Build | `cargo build` | `horus build` | Exists |
| Build release | `cargo build --release` | `horus build --release` | Exists |
| Run | `cargo run` | `horus run` | Exists |
| Run tests | `cargo test` | `horus test` | Exists |
| Run specific test | `cargo test my_test` | `horus test my_test` | Exists |
| Clean | `cargo clean` | `horus clean` | Exists |
| Check | `cargo check` | `horus check` | Exists |
| Update all deps | `cargo update` | `horus update` | Exists |
| Update one dep | `cargo update -p serde` | `horus update serde` | Exists |
| List deps | `cargo tree` | `horus list` | Exists |
| Install Python dep | `pip install numpy` | `horus install numpy` | Exists |
| Uninstall Python dep | `pip uninstall numpy` | `horus remove numpy` | Exists |
| Freeze Python env | `pip freeze` | `horus env freeze` | Exists |
| Run Python script | `python main.py` | `horus run main.py` | Exists |
| Format code | `cargo fmt` | `horus fmt` | **MISSING — add** |
| Lint code | `cargo clippy` | `horus lint` | **MISSING — add** |
| Generate docs | `cargo doc` | `horus doc` | **MISSING — add** |
| Bench | `cargo bench` | `horus bench` | **MISSING — add** |
| Publish | `cargo publish` | `horus publish` | Exists |

Missing commands (`fmt`, `lint`, `doc`, `bench`) should be added as pass-through wrappers that run the underlying tool from `.horus/`. These are simple — just `cargo fmt/clippy/doc/bench` with the right manifest path.

## Shell Integration: cargo/pip Aliases for Horus Projects

Users can still use `cargo` and `pip` commands inside horus projects via shell activation. Aliases only apply inside the horus shell — outside of it, `cargo` and `pip` work normally on non-horus projects.

```
~/my-horus-project/   →  horus shell  →  cargo build = cargo --manifest-path .horus/Cargo.toml
~/my-horus-project/   →  exit         →  cargo build = normal cargo (no alias)
~/some-other-project/ →  cargo build  →  normal cargo (no horus involved)
```

### `horus shell` — Interactive Subshell

```bash
horus shell
# Enters subshell with aliases:
#   cargo build    →  cargo build --manifest-path .horus/Cargo.toml
#   cargo test     →  cargo test --manifest-path .horus/Cargo.toml
#   cargo check    →  cargo check --manifest-path .horus/Cargo.toml
#   cargo fmt      →  cargo fmt --manifest-path .horus/Cargo.toml
#   cargo clippy   →  cargo clippy --manifest-path .horus/Cargo.toml
#   cargo doc      →  cargo doc --manifest-path .horus/Cargo.toml
#   cargo add X    →  horus install X
#   cargo remove X →  horus remove X
#   pip install X  →  horus install X
#   pip uninstall X → horus remove X
#   exit           →  leaves subshell, aliases gone
```

Or inline activation without a subshell:
```bash
eval $(horus env activate)
```

### `.horus/activate.sh` — Scriptable Activation

Generated automatically by `horus new` and `horus init`:

```bash
# .horus/activate.sh — AUTO-GENERATED by horus
# Source this to alias cargo/pip commands for this horus project.
# Only affects the current shell session.

_horus_manifest=".horus/Cargo.toml"

cargo() {
    case "$1" in
        add)    shift; horus install "$@" ;;
        remove) shift; horus remove "$@" ;;
        *)      command cargo --manifest-path "$_horus_manifest" "$@" ;;
    esac
}

pip() {
    case "$1" in
        install)   shift; horus install "$@" ;;
        uninstall) shift; horus remove "$@" ;;
        *)         command pip "$@" ;;
    esac
}

export -f cargo pip
```

### direnv Integration

Users who use direnv can auto-activate per project. `horus new` generates a `.envrc`:

```bash
# .envrc
source .horus/activate.sh
```

direnv activates when you `cd` into the project and deactivates when you leave — so `cargo` and `pip` are aliased only inside horus projects, and work normally everywhere else.

### Implementation

- `horus new` and `horus init`: Generate `.horus/activate.sh` and optionally `.envrc`
- `horus shell`: Spawn `$SHELL` subshell after sourcing `.horus/activate.sh`
- No global state, no PATH modification, no system-wide changes

**Files to add:**
- `commands/shell.rs` — `horus shell` command (spawn subshell with activation)
- Template for `.horus/activate.sh` in `commands/new.rs`

## Risks and Considerations

### Cargo.lock location
`.horus/Cargo.lock` is inside a gitignored directory. For reproducible builds, either:
- Option A: Copy `.horus/Cargo.lock` to root as `horus.lock.cargo` (ugly)
- Option B: Don't gitignore `.horus/Cargo.lock` specifically (`.gitignore` exception)
- Option C: Keep `.horus/Cargo.lock` committed alongside `horus.toml` (needs gitignore rule: `!.horus/Cargo.lock`)
- **Recommended: Option C** — add `!.horus/Cargo.lock` to `.gitignore`

### Workspace support
Some Rust projects use Cargo workspaces with multiple crates. For now, this redesign targets **single-crate horus projects**. Workspace support can be added later via `[workspace]` section in `horus.toml`.

### Import scanning
Currently `horus run` auto-scans imports to detect deps. With explicit `[dependencies]` in `horus.toml`, this becomes optional / a convenience feature. `horus run` could still warn about missing deps detected from imports.

### Existing projects
The `horus migrate` command handles converting existing projects. During transition, `horus build` can detect root `Cargo.toml` without `horus.toml` and suggest running `horus migrate`.

---

## Smart Backend Dispatch — Auto-Detect Everything

### Design Principle

Every horus command is a **smart frontend** that auto-detects the project context and dispatches to the right backend. The user never thinks about which tool to invoke — horus figures it out.

```
horus fmt       →  detects Rust   → cargo fmt
                →  detects Python → ruff format (or black)
                →  detects both   → runs both in parallel

horus test      →  detects Rust   → cargo test from .horus/
                →  detects Python → pytest
                →  detects both   → runs both, merged output

horus install X →  checks registry → horus package
                →  checks crates.io → adds to horus.toml [dependencies] source="crates.io"
                →  checks pypi     → adds to horus.toml [dependencies] source="pypi"
                →  ambiguous       → prompts user
```

### Core Dispatch Module: `src/dispatch.rs`

All smart commands share a single dispatch layer that reuses the existing `detect_languages()` infrastructure:

```rust
use crate::manifest::{detect_languages, Language};

/// Detected project context — what languages, tools, and capabilities are available.
pub struct ProjectContext {
    pub languages: Vec<Language>,
    pub tools: ToolChain,
    pub manifest: Option<HorusManifest>,
    pub workspace_root: PathBuf,
}

/// Available tools detected on the system.
pub struct ToolChain {
    // Rust
    pub cargo: Option<PathBuf>,
    pub rustfmt: Option<PathBuf>,
    pub clippy: bool,               // cargo clippy available?

    // Python
    pub python: Option<String>,      // "python3" or "python"
    pub pip: Option<String>,
    pub ruff: Option<PathBuf>,
    pub black: Option<PathBuf>,
    pub pytest: Option<PathBuf>,
    pub mypy: Option<PathBuf>,
    pub pyright: Option<PathBuf>,

    // Docs
    pub cargo_doc: bool,
    pub sphinx: Option<PathBuf>,
    pub pdoc: Option<PathBuf>,
}

/// Detect full project context. Called once per command, cached.
pub fn detect_context() -> Result<ProjectContext> {
    let workspace_root = find_workspace_root()?.unwrap_or_else(|| PathBuf::from("."));
    let languages = detect_languages(&workspace_root);
    let manifest = HorusManifest::load_from(&workspace_root).ok();
    let tools = detect_toolchain();
    Ok(ProjectContext { languages, tools, manifest, workspace_root })
}

/// Probe system for available tools. Fast — only checks `which`/`command -v`.
fn detect_toolchain() -> ToolChain {
    ToolChain {
        cargo: which("cargo"),
        rustfmt: which("rustfmt"),
        clippy: Command::new("cargo").args(["clippy", "--version"]).output().map(|o| o.status.success()).unwrap_or(false),
        python: detect_python_interpreter().ok(),
        pip: which_str("pip3").or_else(|| which_str("pip")),
        ruff: which("ruff"),
        black: which("black"),
        pytest: which("pytest").or_else(|| which("py.test")),
        mypy: which("mypy"),
        pyright: which("pyright"),
        cargo_doc: true,  // always available with cargo
        sphinx: which("sphinx-build"),
        pdoc: which("pdoc"),
    }
}
```

### Tool Preference Resolution

Auto-detect only. No `[tools]` section in horus.toml — horus finds what's installed and uses it. Zero configuration.

Priority order (first found wins):
- **Formatter**: `ruff` > `black` > skip with suggestion
- **Linter**: `ruff` > `pylint` > `flake8` > skip with suggestion
- **Type checker**: `pyright` > `mypy` > skip (optional, never blocks)
- **Test runner**: `pytest` > `unittest` (stdlib fallback)
- **Doc generator**: `pdoc` > `sphinx` > skip with suggestion

### Parallel Multi-Language Execution

When a project has both Rust and Python, commands run both backends **in parallel** and merge output:

```rust
/// Run a command across all detected languages, in parallel.
fn dispatch_parallel(ctx: &ProjectContext, runners: &[Box<dyn LanguageRunner>]) -> Result<()> {
    let handles: Vec<_> = runners.iter()
        .filter(|r| ctx.languages.contains(&r.language()))
        .map(|r| {
            let r = r.clone();
            std::thread::spawn(move || r.execute())
        })
        .collect();

    let mut any_failed = false;
    for h in handles {
        if let Err(e) = h.join().unwrap() {
            any_failed = true;
            cli_output::error(&format!("{}", e));
        }
    }
    if any_failed { bail!("Some checks failed") }
    Ok(())
}
```

Output is prefixed per-language (same pattern as concurrent `horus run`):

```
[rust]   Formatting 12 files...
[python] Formatting 8 files...
[rust]   ✓ All files formatted
[python] ✓ All files formatted
```

---

## New Commands — Smart Auto-Detect Specifications

### `horus fmt`

Format all source code. Auto-detects language, dispatches to the right formatter.

```bash
horus fmt                    # format all (Rust + Python if both present)
horus fmt --rust             # only Rust
horus fmt --python           # only Python
horus fmt --check            # check only, don't modify (CI mode)
horus fmt src/               # specific directory
horus fmt main.rs nav.py     # specific files
```

**Dispatch logic:**
```
Rust   → cargo fmt --manifest-path .horus/Cargo.toml [--check]
Python → ruff format [--check] . (or black [--check] .)
Both   → parallel(Rust, Python)
```

**Implementation:** `commands/fmt.rs`

```rust
pub fn run_fmt(files: Vec<PathBuf>, check: bool, rust_only: bool, python_only: bool) -> Result<()> {
    let ctx = dispatch::detect_context()?;

    let do_rust = !python_only && ctx.languages.contains(&Language::Rust);
    let do_python = !rust_only && ctx.languages.contains(&Language::Python);

    if do_rust && do_python {
        // Parallel
        let (r_result, p_result) = rayon::join(
            || fmt_rust(&ctx, &files, check),
            || fmt_python(&ctx, &files, check),
        );
        r_result?; p_result?;
    } else if do_rust {
        fmt_rust(&ctx, &files, check)?;
    } else if do_python {
        fmt_python(&ctx, &files, check)?;
    } else {
        cli_output::warn("No Rust or Python files detected");
    }
    Ok(())
}

fn fmt_rust(ctx: &ProjectContext, files: &[PathBuf], check: bool) -> Result<()> {
    let mut cmd = Command::new(ctx.tools.cargo.as_ref().context("cargo not found")?);
    cmd.args(["fmt", "--manifest-path", ".horus/Cargo.toml"]);
    if check { cmd.arg("--check"); }
    // If specific .rs files given, pass them via --
    for f in files.iter().filter(|f| f.extension() == Some("rs".as_ref())) {
        cmd.arg("--").arg(f);
    }
    run_with_prefix("[rust]", cmd)
}

fn fmt_python(ctx: &ProjectContext, files: &[PathBuf], check: bool) -> Result<()> {
    let tool = ctx.tools.ruff.as_ref()
        .or(ctx.tools.black.as_ref())
        .context("No Python formatter found. Install: pip install ruff")?;

    let tool_name = tool.file_name().unwrap().to_str().unwrap();
    let mut cmd = Command::new(tool);

    match tool_name {
        "ruff" => {
            cmd.arg("format");
            if check { cmd.arg("--check"); }
        }
        "black" => {
            if check { cmd.arg("--check"); }
        }
        _ => {}
    }

    let py_files: Vec<_> = files.iter().filter(|f| f.extension() == Some("py".as_ref())).collect();
    if py_files.is_empty() { cmd.arg("."); } else { cmd.args(py_files); }

    run_with_prefix("[python]", cmd)
}
```

### `horus lint`

Lint all source code. Auto-detects language, dispatches to the right linter.

```bash
horus lint                   # lint all (clippy + ruff if both present)
horus lint --rust            # only Rust
horus lint --python          # only Python
horus lint --fix             # auto-fix where possible
horus lint --strict          # treat warnings as errors
horus lint src/              # specific directory
```

**Dispatch logic:**
```
Rust   → cargo clippy --manifest-path .horus/Cargo.toml -- -D warnings [if --strict]
Python → ruff check [--fix] . (or pylint .)
Both   → parallel(Rust, Python)
```

**Implementation:** `commands/lint.rs`

```rust
pub fn run_lint(files: Vec<PathBuf>, fix: bool, strict: bool, rust_only: bool, python_only: bool) -> Result<()> {
    let ctx = dispatch::detect_context()?;

    let do_rust = !python_only && ctx.languages.contains(&Language::Rust);
    let do_python = !rust_only && ctx.languages.contains(&Language::Python);

    // Same parallel pattern as fmt
    if do_rust { lint_rust(&ctx, &files, fix, strict)?; }
    if do_python { lint_python(&ctx, &files, fix, strict)?; }
    Ok(())
}

fn lint_rust(ctx: &ProjectContext, _files: &[PathBuf], fix: bool, strict: bool) -> Result<()> {
    ensure!(ctx.tools.clippy, "cargo clippy not available. Install: rustup component add clippy");
    let mut cmd = Command::new(ctx.tools.cargo.as_ref().unwrap());
    cmd.args(["clippy", "--manifest-path", ".horus/Cargo.toml"]);
    if fix { cmd.arg("--fix").arg("--allow-dirty"); }
    if strict { cmd.arg("--").arg("-D").arg("warnings"); }
    run_with_prefix("[rust]", cmd)
}

fn lint_python(ctx: &ProjectContext, files: &[PathBuf], fix: bool, _strict: bool) -> Result<()> {
    let tool = ctx.tools.ruff.as_ref()
        .context("No Python linter found. Install: pip install ruff")?;

    let mut cmd = Command::new(tool);
    cmd.arg("check");
    if fix { cmd.arg("--fix"); }

    let py_files: Vec<_> = files.iter().filter(|f| f.extension() == Some("py".as_ref())).collect();
    if py_files.is_empty() { cmd.arg("."); } else { cmd.args(py_files); }

    run_with_prefix("[python]", cmd)
}
```

### `horus test` — Extended for Python

Currently only does `cargo test`. Extended to auto-detect and run pytest too.

```bash
horus test                        # run all tests (cargo test + pytest)
horus test my_test                # filter by name (both runners)
horus test --rust                 # only Rust tests
horus test --python               # only Python tests
horus test --sim                  # simulation mode (existing)
horus test --parallel             # multi-threaded (existing)
horus test --coverage             # generate coverage report
horus test --integration          # integration tests only (existing)
```

**Dispatch logic:**
```
Rust   → cargo test --manifest-path .horus/Cargo.toml [filter] [flags]
Python → pytest [filter] [-v] [--cov] (from .horus/ venv context)
Both   → sequential (Rust first, then Python — test output is cleaner sequential)
```

**Changes to `commands/test.rs`:**

```rust
pub struct TestConfig {
    pub filter: Option<String>,
    pub release: bool,
    pub nocapture: bool,
    pub test_threads: Option<usize>,
    pub parallel: bool,
    pub simulation: bool,
    pub integration: bool,
    pub no_build: bool,
    pub verbose: bool,
    pub coverage: bool,          // NEW
    pub rust_only: bool,         // NEW
    pub python_only: bool,       // NEW
}

pub fn run_tests(cfg: TestConfig) -> Result<()> {
    let ctx = dispatch::detect_context()?;

    let do_rust = !cfg.python_only && ctx.languages.contains(&Language::Rust);
    let do_python = !cfg.rust_only && ctx.languages.contains(&Language::Python);

    let mut exit_code = 0;

    if do_rust {
        cli_output::header("Running Rust tests");
        // existing cargo test logic, unchanged
        let code = test_rust(&ctx, &cfg)?;
        exit_code = exit_code.max(code);
    }

    if do_python {
        cli_output::header("Running Python tests");
        let code = test_python(&ctx, &cfg)?;
        exit_code = exit_code.max(code);
    }

    if exit_code != 0 {
        bail!("Tests failed with exit code {}", exit_code);
    }
    Ok(())
}

fn test_python(ctx: &ProjectContext, cfg: &TestConfig) -> Result<i32> {
    let pytest = ctx.tools.pytest.as_ref()
        .context("pytest not found. Install: pip install pytest")?;

    let mut cmd = Command::new(pytest);
    if cfg.verbose { cmd.arg("-v"); }
    if cfg.coverage { cmd.args(["--cov", ".", "--cov-report", "term-missing"]); }
    if let Some(ref f) = cfg.filter { cmd.args(["-k", f]); }

    // Set PYTHONPATH to include .horus/packages/
    let pythonpath = ctx.workspace_root.join(".horus/packages");
    cmd.env("PYTHONPATH", &pythonpath);

    let status = cmd.status()?;
    Ok(status.code().unwrap_or(1))
}
```

### `horus doc`

Generate documentation. Auto-detects language, dispatches to the right doc generator.

```bash
horus doc                    # generate docs for all languages
horus doc --rust             # only Rust docs
horus doc --python           # only Python docs
horus doc --open             # open in browser after generating
horus doc --no-deps          # skip dependency docs (Rust)
```

**Dispatch logic:**
```
Rust   → cargo doc --manifest-path .horus/Cargo.toml [--open] [--no-deps]
Python → pdoc --html -o .horus/doc/python . (or sphinx-build)
Both   → parallel(Rust, Python), then open index if --open
```

**Implementation:** `commands/doc.rs`

```rust
pub fn run_doc(rust_only: bool, python_only: bool, open: bool, no_deps: bool) -> Result<()> {
    let ctx = dispatch::detect_context()?;

    let do_rust = !python_only && ctx.languages.contains(&Language::Rust);
    let do_python = !rust_only && ctx.languages.contains(&Language::Python);

    if do_rust { doc_rust(&ctx, open && !do_python, no_deps)?; }
    if do_python { doc_python(&ctx, open && !do_rust)?; }

    if open && do_rust && do_python {
        // Open a combined index pointing to both
        let index = ctx.workspace_root.join(".horus/doc/index.html");
        generate_combined_index(&index, do_rust, do_python)?;
        open_browser(&index)?;
    }
    Ok(())
}
```

### `horus bench`

Run benchmarks. Auto-detects language, dispatches to the right benchmark runner.

```bash
horus bench                  # run all benchmarks
horus bench --rust           # only Rust
horus bench --python         # only Python
horus bench my_bench         # filter by name
horus bench --save           # save results for comparison
horus bench --compare        # compare against last saved baseline
```

**Dispatch logic:**
```
Rust   → cargo bench --manifest-path .horus/Cargo.toml [filter]
Python → pytest --benchmark . (or python -m pytest with pytest-benchmark)
Both   → sequential(Rust, Python)
```

### `horus doctor`

Full ecosystem health check. Inspects every layer — toolchain, libraries, system, hardware, network, project state — and reports actionable fixes.

```bash
horus doctor                 # summary (1 line per category, 10 lines total)
horus doctor --verbose       # full detailed output (every check expanded)
horus doctor --fix           # auto-fix what it can (clean stale state, regenerate files)
horus doctor --category sys  # deep dive on one category
horus doctor --json          # machine-readable output (for CI)
```

**Default output** (`horus doctor`):

```
horus doctor
  horus       ✓ 0.1.9 (all libs match)
  rust        ✓ cargo 1.82.0, clippy, rustfmt
  python      ✓ python 3.11.5, ruff, pytest
  system      ! 2 stale SHM namespaces
  hardware    ✓ 3 devices, all groups ok
  network     ✓ registry reachable [204ms]
  plugins     ! 1 incompatible
  project     ✓ my-robot (12 deps, lockfile ok)
  workspaces  ✓ 12 registered, cache 847 MB
  ─────────
  7 ok, 2 warnings — run with --verbose for details
```

**Verbose output** (`horus doctor --verbose`):

```
═══════════════════════════════════════════════════════════
  horus doctor — ecosystem health check
═══════════════════════════════════════════════════════════

── HORUS ──────────────────────────────────────────────────
  ✓ horus CLI             0.1.9
  ✓ horus_core            0.1.9 (matches CLI)
  ✓ horus_library         0.1.9 (matches CLI)
  ✓ horus_macros          0.1.9 (matches CLI)
  ✓ horus_py              0.1.9 (pip: horus-robotics 0.1.9)
  ✗ horus-sim3d           not installed
    → Install: cd ~/softmata/horus-sim3d && cargo install --path .

── RUST TOOLCHAIN ─────────────────────────────────────────
  ✓ rustup                1.28.0
  ✓ rustc                 1.82.0 (stable)
  ✓ cargo                 1.82.0
  ✓ rustfmt               installed
  ✓ clippy                installed
  ✗ cargo-audit           not installed (optional)
    → Install: cargo install cargo-audit
  ✗ cargo-tarpaulin       not installed (optional — for coverage)
    → Install: cargo install cargo-tarpaulin

── PYTHON TOOLCHAIN ───────────────────────────────────────
  ✓ python                3.11.5
  ✓ pip                   24.0
  ✓ maturin               1.7.0 (for horus_py builds)
  ✓ ruff                  0.8.2 (formatter + linter)
  ✓ pytest                8.3.0
  ✗ pytest-cov            not installed (for coverage)
    → Install: pip install pytest-cov
  ✗ mypy                  not installed (optional)
    → Install: pip install mypy

── SYSTEM ─────────────────────────────────────────────────
  ✓ OS                    Linux 6.6.87 (WSL2)
  ✓ arch                  x86_64
  ✓ /dev/shm              accessible (tmpfs, 3.9 GB free)
  ✗ shared memory         2 stale namespaces (14 MB)
    → Fix: horus clean --shm
  ✓ RT scheduling         limits configured (/etc/security/limits.d/99-horus-realtime.conf)
  ✓ CPU governor          performance
  ✓ disk space            42 GB free (project), 18 GB free (~/.horus/)
  ✗ build cache           2.1 GB (target/)
    → Suggestion: horus clean (frees 2.1 GB)

── HARDWARE ───────────────────────────────────────────────
  ✓ /dev/video0           camera (v4l2, 640x480)
  ✓ /dev/ttyUSB0          serial (ftdi, 115200)
  ✗ /dev/i2c-1            not found (needed by bno055-imu feature)
    → Enable: sudo raspi-config → I2C → Enable
  ✓ user groups           dialout ✓, video ✓, i2c ✗, gpio ✗
    → Fix: sudo usermod -aG i2c,gpio $USER

── NETWORK & REGISTRY ────────────────────────────────────
  ✓ registry              reachable [204ms] (https://registry.horus-registry.dev)
  ✓ registry API          v1.2.0 (compatible)
  ✓ auth                  logged in as @neos (key scope: full)
  ✓ signing key           ~/.horus/keys/signing_key (ed25519)
  ✓ GitHub                authenticated (token valid)

── PLUGINS ────────────────────────────────────────────────
  ✓ 3 plugins installed   (2 global, 1 project)
  ✓ horus-gazebo          1.0.0 (enabled, verified)
  ✗ horus-ros-bridge      0.9.0 (incompatible with horus 0.1.9, needs ≤0.1.8)
    → Fix: horus update horus-ros-bridge
  ✓ my-custom-tool        0.2.0 (enabled, local)

── PROJECT (my-robot) ────────────────────────────────────
  ✓ horus.toml            valid (edition 1)
  ✓ languages             Rust, Python
  ✓ dependencies          12 total (8 Rust, 3 Python, 1 registry)
  ✓ .horus/               exists
  ✗ .horus/Cargo.toml     stale (horus.toml changed since last build)
    → Fix: horus build
  ✓ horus.lock            up to date
  ✗ outdated deps         3 packages have newer versions
    → Run: horus deps outdated
  ✗ security              1 known vulnerability in transitive deps
    → Run: horus deps audit

── WORKSPACES ─────────────────────────────────────────────
  ✓ 12 registered workspaces in ~/.horus/workspaces.json
  ✗ 3 stale workspaces    (directories no longer exist)
    → Fix: horus doctor --fix (cleans stale entries)
  ✓ cache                 847 MB (23 packages)
  ✗ orphaned cache        2 packages not referenced by any workspace
    → Fix: horus cache clean

═══════════════════════════════════════════════════════════
  Summary: 28 passed, 9 warnings, 2 failures
  Run `horus doctor --fix` to auto-resolve 6 issues
═══════════════════════════════════════════════════════════
```

**Categories** (each can be run independently with `--category`):

| Category | Flag | What it checks |
|----------|------|----------------|
| `horus` | `--category horus` | CLI version, core lib versions, horus_py, sim3d, version mismatches |
| `rust` | `--category rust` | rustup, rustc, cargo, rustfmt, clippy, cargo-audit, cargo-tarpaulin |
| `python` | `--category python` | python, pip, maturin, ruff, black, pytest, pytest-cov, mypy, pyright |
| `sys` | `--category sys` | OS, arch, /dev/shm, RT config, CPU governor, disk, stale SHM, build cache size |
| `hw` | `--category hw` | /dev/ devices, user groups, driver availability, feature enablement |
| `net` | `--category net` | Registry reachability + API version, auth, signing keys, GitHub token |
| `plugins` | `--category plugins` | Installed plugins, version compatibility, verified status |
| `project` | `--category project` | horus.toml, .horus/ state, deps freshness, lockfile, security audit |
| `workspaces` | `--category workspaces` | Workspace registry, stale entries, cache integrity, orphaned packages |

**`--fix` auto-resolves:**
- Stale SHM namespaces → `clean_shared_memory()`
- Stale workspace entries → remove from workspaces.json
- Orphaned cache → `horus cache clean`
- Missing Python tools → `pip install ruff pytest pytest-cov`
- Missing Rust components → `rustup component add rustfmt clippy`
- Stale .horus/Cargo.toml → regenerate from horus.toml
- Incompatible plugins → `horus update <plugin>`

**Implementation:** `commands/doctor.rs`

```rust
pub struct DoctorConfig {
    pub json: bool,
    pub fix: bool,
    pub verbose: bool,
    pub category: Option<String>,
}

pub fn run_doctor(cfg: DoctorConfig) -> Result<()> {
    let ctx = dispatch::detect_context()?;
    let mut report = DoctorReport::new();

    let categories = match cfg.category.as_deref() {
        Some(c) => vec![c.to_string()],
        None => vec!["horus", "rust", "python", "sys", "hw", "net", "plugins", "project", "workspaces"]
            .into_iter().map(String::from).collect(),
    };

    for cat in &categories {
        match cat.as_str() {
            "horus"      => check_horus_ecosystem(&ctx, &mut report, cfg.fix)?,
            "rust"       => check_rust_toolchain(&ctx, &mut report, cfg.fix)?,
            "python"     => check_python_toolchain(&ctx, &mut report, cfg.fix)?,
            "sys"        => check_system(&ctx, &mut report, cfg.fix)?,
            "hw"         => check_hardware(&ctx, &mut report)?,
            "net"        => check_network(&ctx, &mut report)?,
            "plugins"    => check_plugins(&ctx, &mut report, cfg.fix)?,
            "project"    => check_project(&ctx, &mut report, cfg.fix)?,
            "workspaces" => check_workspaces(&ctx, &mut report, cfg.fix)?,
            _ => bail!("Unknown category: {}", cat),
        }
    }

    if cfg.verbose {
        report.print_verbose(cfg.json);
    } else {
        report.print_summary(cfg.json);  // 1 line per category
    }
    std::process::exit(if report.has_failures() { 1 } else { 0 });
}

/// Check all horus crate versions match the CLI version.
fn check_horus_ecosystem(ctx: &ProjectContext, report: &mut DoctorReport, fix: bool) -> Result<()> {
    let cli_version = env!("CARGO_PKG_VERSION");
    report.section("HORUS");
    report.ok("horus CLI", cli_version);

    // Check installed library version matches CLI
    let installed_ver = std::fs::read_to_string(
        dirs::home_dir().unwrap().join(".horus/installed_version")
    ).unwrap_or_default().trim().to_string();

    if installed_ver.is_empty() {
        report.warn("installed version", "~/.horus/installed_version missing — run: horus upgrade");
    } else if installed_ver != cli_version {
        report.fail("version mismatch",
            &format!("CLI={} but installed libs={} — run: horus upgrade", cli_version, installed_ver));
    } else {
        report.ok("horus_core", &format!("{} (matches CLI)", installed_ver));
        report.ok("horus_library", &format!("{} (matches CLI)", installed_ver));
        report.ok("horus_macros", &format!("{} (matches CLI)", installed_ver));
    }

    // Check horus_py (Python bindings)
    if let Some(ref py) = ctx.tools.python {
        match tool_output(py, &["-c", "import horus; print(horus.__version__)"]) {
            Ok(ver) => {
                if ver.trim() == cli_version {
                    report.ok("horus_py", &format!("{} (pip: horus-robotics)", ver.trim()));
                } else {
                    report.warn("horus_py", &format!(
                        "{} (outdated, CLI is {}) — run: pip install --upgrade horus-robotics", ver.trim(), cli_version));
                }
            }
            Err(_) => report.info("horus_py", "not installed — install: pip install horus-robotics"),
        }
    }

    // Check horus-sim3d
    match which("sim3d") {
        Some(_) => report.ok("horus-sim3d", "installed"),
        None => report.info("horus-sim3d", "not installed (optional — for 3D simulation)"),
    }

    Ok(())
}

/// Check system: SHM, disk, RT config, build cache.
fn check_system(ctx: &ProjectContext, report: &mut DoctorReport, fix: bool) -> Result<()> {
    report.section("SYSTEM");

    // OS info
    report.ok("OS", &format!("{} {}", std::env::consts::OS, std::env::consts::ARCH));

    // Shared memory
    let shm_path = Path::new("/dev/shm");
    if shm_path.exists() {
        let statvfs = nix::sys::statvfs::statvfs(shm_path)?;
        let free_mb = (statvfs.blocks_available() * statvfs.fragment_size() as u64) / (1024 * 1024);
        report.ok("/dev/shm", &format!("accessible ({} MB free)", free_mb));
    } else {
        report.fail("/dev/shm", "not accessible — IPC will not work");
    }

    // Stale SHM namespaces
    let stale_ns = list_stale_namespaces()?;
    if !stale_ns.is_empty() {
        let total_bytes: u64 = stale_ns.iter().map(|n| n.size_bytes).sum();
        report.warn("shared memory",
            &format!("{} stale namespaces ({}) — fix: horus clean --shm",
                stale_ns.len(), format_bytes(total_bytes)));
        if fix {
            clean_shared_memory(false, false)?;
            report.fixed("shared memory", "cleaned stale namespaces");
        }
    } else {
        report.ok("shared memory", "no stale namespaces");
    }

    // RT config
    let rt_conf = Path::new("/etc/security/limits.d/99-horus-realtime.conf");
    if rt_conf.exists() {
        report.ok("RT scheduling", "limits configured");
    } else {
        report.info("RT scheduling", "not configured — run: sudo ./scripts/setup-realtime.sh (optional)");
    }

    // Disk space
    let home_horus = dirs::home_dir().unwrap().join(".horus");
    if home_horus.exists() {
        let cache_size = dir_size(&home_horus.join("cache"))?;
        report.ok("~/.horus/cache", &format!("{}", format_bytes(cache_size)));
    }

    // Build cache size
    let target_dir = ctx.workspace_root.join("target");
    if target_dir.exists() {
        let target_size = dir_size(&target_dir)?;
        if target_size > 2 * 1024 * 1024 * 1024 { // > 2 GB
            report.warn("build cache", &format!("{} — suggestion: horus clean", format_bytes(target_size)));
        } else {
            report.ok("build cache", &format!("{}", format_bytes(target_size)));
        }
    }

    Ok(())
}

/// Check hardware devices, user groups, driver features.
fn check_hardware(ctx: &ProjectContext, report: &mut DoctorReport) -> Result<()> {
    report.section("HARDWARE");

    // Scan /dev/ for common robotics devices
    let devices = [
        ("/dev/video*",   "camera",   "video"),
        ("/dev/ttyUSB*",  "serial",   "dialout"),
        ("/dev/ttyACM*",  "serial",   "dialout"),
        ("/dev/i2c-*",    "i2c",      "i2c"),
        ("/dev/spidev*",  "spi",      "spi"),
        ("/dev/gpiomem",  "gpio",     "gpio"),
        ("/dev/input/js*","joystick", "input"),
    ];

    for (pattern, device_type, group) in &devices {
        let matches = glob::glob(pattern).ok().map(|g| g.filter_map(|p| p.ok()).collect::<Vec<_>>()).unwrap_or_default();
        if !matches.is_empty() {
            // Check user group membership
            let in_group = is_user_in_group(group);
            if in_group {
                report.ok(&matches[0].display().to_string(),
                    &format!("{} (group {} ✓)", device_type, group));
            } else {
                report.warn(&matches[0].display().to_string(),
                    &format!("{} (missing group {}) — fix: sudo usermod -aG {} $USER", device_type, group, group));
            }
        }
    }

    // Cross-reference with enabled features from horus.toml
    if let Some(ref manifest) = ctx.manifest {
        for (feature, dep) in system_deps::SYSTEM_DEPS {
            if manifest.enable.contains(&feature.to_string()) {
                // Check if device files exist
                for dev in dep.device_files {
                    let found = glob::glob(dev).ok().map(|g| g.count() > 0).unwrap_or(false);
                    if !found {
                        report.warn(feature, &format!("feature enabled but {} not found", dev));
                    }
                }
            }
        }
    }

    Ok(())
}

/// Check network: registry, auth, signing keys.
fn check_network(ctx: &ProjectContext, report: &mut DoctorReport) -> Result<()> {
    report.section("NETWORK & REGISTRY");

    // Registry health
    let start = std::time::Instant::now();
    match RegistryClient::new().health_check() {
        Ok(api_version) => {
            let latency = start.elapsed().as_millis();
            report.ok("registry", &format!("reachable [{}ms] (API {})", latency, api_version));
        }
        Err(e) => report.fail("registry", &format!("unreachable: {}", e)),
    }

    // Auth
    match load_auth_config() {
        Ok(auth) => {
            report.ok("auth", &format!("logged in as @{} (scope: {})", auth.username, auth.scope));
            // Check token validity
            match RegistryClient::new().whoami(&auth.api_key) {
                Ok(_) => report.ok("auth token", "valid"),
                Err(_) => report.warn("auth token", "expired or invalid — run: horus auth login"),
            }
        }
        Err(_) => report.info("auth", "not logged in — run: horus auth login"),
    }

    // Signing key
    let key_dir = dirs::home_dir().unwrap().join(".horus/keys");
    let has_private = key_dir.join("signing_key").exists();
    let has_public = key_dir.join("signing_key.pub").exists();
    match (has_private, has_public) {
        (true, true) => report.ok("signing key", "keypair present (ed25519)"),
        (true, false) => report.warn("signing key", "private key found but no public key — run: horus keygen"),
        (false, _) => report.info("signing key", "not found — run: horus keygen (needed for publishing)"),
    }

    Ok(())
}

/// Check plugins: installed, version compat, verification status.
fn check_plugins(ctx: &ProjectContext, report: &mut DoctorReport, fix: bool) -> Result<()> {
    report.section("PLUGINS");

    let global_registry = load_plugin_registry(PluginScope::Global)?;
    let project_registry = load_plugin_registry(PluginScope::Project).ok();

    let total = global_registry.plugins.len()
        + project_registry.as_ref().map(|r| r.plugins.len()).unwrap_or(0);

    if total == 0 {
        report.info("plugins", "none installed");
        return Ok(());
    }

    report.ok("plugins", &format!("{} installed", total));

    let cli_version = semver::Version::parse(env!("CARGO_PKG_VERSION"))?;

    for (name, entry) in global_registry.plugins.iter()
        .chain(project_registry.iter().flat_map(|r| r.plugins.iter()))
    {
        // Version compatibility check
        if let Some(ref req) = entry.compatibility.cli_version_req {
            if let Ok(req) = semver::VersionReq::parse(req) {
                if !req.matches(&cli_version) {
                    report.warn(name, &format!(
                        "{} (incompatible: needs horus {}, have {}) — fix: horus update {}",
                        entry.version, req, cli_version, name));
                    if fix {
                        // Try to update
                        let _ = run_update(Some(name.clone()), true, false);
                        report.fixed(name, "updated");
                    }
                    continue;
                }
            }
        }

        // Disabled check
        if global_registry.disabled.contains_key(name) {
            report.info(name, &format!("{} (disabled)", entry.version));
            continue;
        }

        // Binary exists check
        if !entry.binary.exists() {
            report.fail(name, &format!("{} (binary missing: {}) — fix: horus install {}",
                entry.version, entry.binary.display(), name));
            continue;
        }

        report.ok(name, &format!("{} (verified, enabled)", entry.version));
    }

    Ok(())
}

/// Check project: horus.toml, .horus/ state, deps, lockfile, security.
fn check_project(ctx: &ProjectContext, report: &mut DoctorReport, fix: bool) -> Result<()> {
    report.section("PROJECT");

    let manifest = match ctx.manifest {
        Some(ref m) => m,
        None => {
            report.info("project", "no horus.toml in current directory");
            return Ok(());
        }
    };

    report.ok("horus.toml", &format!("{} (edition {})", manifest.package.name, manifest.package.edition));
    report.ok("languages", &ctx.languages.iter().map(|l| format!("{:?}", l)).collect::<Vec<_>>().join(", "));

    // Dependency count
    let rust_deps = manifest.dependencies.iter()
        .filter(|(_, v)| matches!(v.source(), Some("crates.io") | None)).count();
    let py_deps = manifest.dependencies.iter()
        .filter(|(_, v)| v.source() == Some("pypi")).count();
    let reg_deps = manifest.dependencies.iter()
        .filter(|(_, v)| v.source() == Some("registry")).count();
    report.ok("dependencies", &format!("{} total ({} Rust, {} Python, {} registry)",
        manifest.dependencies.len(), rust_deps, py_deps, reg_deps));

    // .horus/ directory state
    let horus_dir = ctx.workspace_root.join(".horus");
    if !horus_dir.exists() {
        report.warn(".horus/", "missing — run: horus build");
        return Ok(());
    }

    // Generated Cargo.toml freshness
    let generated_cargo = horus_dir.join("Cargo.toml");
    if generated_cargo.exists() {
        if is_generated_stale(&generated_cargo, &ctx.workspace_root.join("horus.toml"))? {
            report.warn(".horus/Cargo.toml", "stale (horus.toml changed) — fix: horus build");
            if fix {
                cargo_gen::generate(&manifest)?;
                report.fixed(".horus/Cargo.toml", "regenerated");
            }
        } else {
            report.ok(".horus/Cargo.toml", "up to date");
        }
    }

    // Lockfile
    let lockfile = ctx.workspace_root.join("horus.lock");
    if lockfile.exists() {
        if is_lockfile_current(&lockfile, &manifest)? {
            report.ok("horus.lock", "up to date");
        } else {
            report.warn("horus.lock", "out of sync — run: horus build");
        }
    } else {
        report.warn("horus.lock", "missing — run: horus build (generates lockfile)");
    }

    // Outdated deps (skip in --quick)
    let outdated = check_outdated_deps(&manifest)?;
    if !outdated.is_empty() {
        report.warn("outdated deps", &format!("{} packages have newer versions — run: horus deps outdated", outdated.len()));
    }

    // Security audit (skip in --quick)
    let vulns = check_vulnerability_audit(&ctx)?;
    if !vulns.is_empty() {
        report.fail("security", &format!("{} known vulnerabilities — run: horus deps audit", vulns.len()));
    }

    Ok(())
}

/// Check workspace registry and cache.
fn check_workspaces(ctx: &ProjectContext, report: &mut DoctorReport, fix: bool) -> Result<()> {
    report.section("WORKSPACES");

    let ws_path = dirs::home_dir().unwrap().join(".horus/workspaces.json");
    if !ws_path.exists() {
        report.info("workspaces", "no workspace registry");
        return Ok(());
    }

    let workspaces: Vec<RegisteredWorkspace> = serde_json::from_str(
        &std::fs::read_to_string(&ws_path)?
    )?;

    let total = workspaces.len();
    let stale: Vec<_> = workspaces.iter().filter(|w| !Path::new(&w.path).exists()).collect();

    report.ok("registered", &format!("{} workspaces", total));

    if !stale.is_empty() {
        report.warn("stale workspaces", &format!(
            "{} directories no longer exist", stale.len()));
        if fix {
            // Remove stale entries
            let cleaned: Vec<_> = workspaces.into_iter()
                .filter(|w| Path::new(&w.path).exists())
                .collect();
            std::fs::write(&ws_path, serde_json::to_string_pretty(&cleaned)?)?;
            report.fixed("stale workspaces", &format!("removed {} entries", stale.len()));
        }
    }

    // Cache integrity
    let cache_dir = dirs::home_dir().unwrap().join(".horus/cache");
    if cache_dir.exists() {
        let cache_size = dir_size(&cache_dir)?;
        let pkg_count = std::fs::read_dir(&cache_dir)?.count();
        report.ok("cache", &format!("{} ({} packages)", format_bytes(cache_size), pkg_count));

        // Check for orphaned packages (not referenced by any workspace)
        let orphaned = find_orphaned_cache_entries(&cache_dir)?;
        if !orphaned.is_empty() {
            let orphan_size: u64 = orphaned.iter().map(|p| dir_size(p).unwrap_or(0)).sum();
            report.warn("orphaned cache", &format!(
                "{} packages ({}) not used by any workspace — fix: horus cache clean",
                orphaned.len(), format_bytes(orphan_size)));
            if fix {
                for p in &orphaned { let _ = std::fs::remove_dir_all(p); }
                report.fixed("orphaned cache", &format!("removed {} packages", orphaned.len()));
            }
        }
    }

    Ok(())
}
```

### `horus shell`

Enter an activated subshell where `cargo` and `pip` are aliased to work through `.horus/`.

```bash
horus shell                  # enter activated subshell
horus shell --print          # print activation script (for eval)
horus shell --fish           # fish shell activation
horus shell --zsh            # zsh activation (default on macOS)
```

**Implementation:** `commands/shell.rs`

```rust
pub fn run_shell(print_only: bool, shell_type: Option<String>) -> Result<()> {
    let ctx = dispatch::detect_context()?;
    let manifest_path = ctx.workspace_root.join(".horus/Cargo.toml");
    let activate_script = generate_activate_script(&manifest_path, &shell_type)?;

    if print_only {
        // For: eval $(horus shell --print)
        println!("{}", activate_script);
        return Ok(());
    }

    // Write activation script
    let activate_path = ctx.workspace_root.join(".horus/activate.sh");
    std::fs::write(&activate_path, &activate_script)?;

    // Spawn subshell
    let shell = std::env::var("SHELL").unwrap_or_else(|_| "/bin/bash".to_string());
    let mut cmd = Command::new(&shell);
    cmd.arg("--rcfile").arg(&activate_path);
    cmd.env("HORUS_SHELL", "1");        // marker for prompt customization
    cmd.env("HORUS_PROJECT", ctx.manifest.as_ref().map(|m| m.package.name.as_str()).unwrap_or("horus"));

    cli_output::info(&format!("Entering horus shell — cargo/pip commands are aliased. Type 'exit' to leave."));
    let status = cmd.status()?;
    std::process::exit(status.code().unwrap_or(0));
}
```

### `horus migrate`

Convert an existing project (with root Cargo.toml/pyproject.toml) to unified horus.toml.

```bash
horus migrate                # auto-detect and migrate
horus migrate --dry-run      # show what would change
horus migrate --backup       # keep .bak copies (default: true)
```

**Implementation:** `commands/migrate.rs`

```rust
pub fn run_migrate(dry_run: bool, backup: bool) -> Result<()> {
    let cwd = std::env::current_dir()?;

    // Check what exists
    let has_cargo = cwd.join("Cargo.toml").exists();
    let has_pyproject = cwd.join("pyproject.toml").exists();
    let has_horus = cwd.join("horus.toml").exists();

    if !has_cargo && !has_pyproject {
        bail!("Nothing to migrate — no Cargo.toml or pyproject.toml found");
    }

    let mut deps: BTreeMap<String, DependencyValue> = BTreeMap::new();

    // Phase 1: Read existing Cargo.toml deps
    if has_cargo {
        let cargo_deps = parse_cargo_toml_deps(&cwd.join("Cargo.toml"))?;
        for (name, spec) in cargo_deps {
            if is_horus_core_crate(&name) { continue; } // skip horus path deps
            deps.insert(name, DependencyValue::Detailed(DetailedDependency {
                version: Some(spec.version),
                source: Some("crates.io".into()),
                features: spec.features,
                ..Default::default()
            }));
        }
        cli_output::info(&format!("Read {} Rust dependencies from Cargo.toml", deps.len()));
    }

    // Phase 2: Read existing pyproject.toml deps
    if has_pyproject {
        let py_deps = parse_pyproject_deps(&cwd.join("pyproject.toml"))?;
        for (name, version) in py_deps {
            deps.insert(name, DependencyValue::Detailed(DetailedDependency {
                version: Some(version),
                source: Some("pypi".into()),
                ..Default::default()
            }));
        }
    }

    if dry_run {
        cli_output::header("Dry run — would make these changes:");
        println!("  Add {} dependencies to horus.toml [dependencies]", deps.len());
        if has_cargo { println!("  Move Cargo.toml → .horus/Cargo.toml.bak"); }
        if has_pyproject { println!("  Move pyproject.toml → .horus/pyproject.toml.bak"); }
        return Ok(());
    }

    // Phase 3: Create/update horus.toml
    let mut manifest = if has_horus {
        HorusManifest::load_from(&cwd)?
    } else {
        HorusManifest::default_for(&cwd)?
    };
    manifest.dependencies = deps;
    manifest.save_to(&cwd.join("horus.toml"))?;

    // Phase 4: Move native build files to backup
    if has_cargo && backup {
        std::fs::rename(cwd.join("Cargo.toml"), cwd.join(".horus/Cargo.toml.bak"))?;
    } else if has_cargo {
        std::fs::remove_file(cwd.join("Cargo.toml"))?;
    }

    if has_pyproject && backup {
        std::fs::rename(cwd.join("pyproject.toml"), cwd.join(".horus/pyproject.toml.bak"))?;
    } else if has_pyproject {
        std::fs::remove_file(cwd.join("pyproject.toml"))?;
    }

    // Phase 5: Move src/main.rs → main.rs if needed
    if cwd.join("src/main.rs").exists() && !cwd.join("main.rs").exists() {
        std::fs::rename(cwd.join("src/main.rs"), cwd.join("main.rs"))?;
        cli_output::info("Moved src/main.rs → main.rs");
    }

    cli_output::success("Migration complete. Run 'horus build' to verify.");
    Ok(())
}
```

### `horus deps` — Dependency Insight Subcommands

New subcommand group for dependency management DX:

```bash
horus deps tree              # show full dependency tree
horus deps tree --rust       # Rust deps only
horus deps tree --python     # Python deps only
horus deps why serde         # why is this dep needed? (reverse tree)
horus deps outdated          # show deps with newer versions available
horus deps audit             # security vulnerability scan
horus deps duplicates        # show duplicate transitive deps
```

**Dispatch logic:**
```
tree    → Rust: cargo tree --manifest-path .horus/Cargo.toml
          Python: pipdeptree (or pip list --format=tree)
          Registry: list from horus.toml + resolve transitives
why     → Rust: cargo tree -i <pkg> --manifest-path .horus/Cargo.toml
          Python: pipdeptree --reverse <pkg>
outdated→ Rust: cargo outdated (if installed) or cargo update --dry-run
          Python: pip list --outdated
          Registry: check registry API for newer versions
audit   → Rust: cargo audit (if installed)
          Python: pip-audit (if installed)
          Registry: check registry vulnerability reports
```

### `horus upgrade`

Upgrade the entire horus ecosystem — CLI, libraries, Python bindings, simulator, toolchains, plugins, and project dependencies. Not just "self-update" — everything.

```bash
horus upgrade                # upgrade horus ecosystem (interactive, shows plan first)
horus upgrade --check        # show what would be upgraded, don't do anything
horus upgrade --yes          # skip confirmation prompts
horus upgrade --cli          # only upgrade horus CLI + core libraries + horus_py
horus upgrade --plugins      # only upgrade installed plugins
horus upgrade --deps         # only upgrade project dependencies
horus upgrade --version 0.6  # target specific horus version
```

**Not horus's job** (users manage their own toolchains):
- `rustup update` — user runs this themselves
- `pip install --upgrade ruff` — user manages their Python tools
- `cargo install cargo-audit` — user installs their own optional tools

**Output:**

```
═══════════════════════════════════════════════════════════
  horus upgrade — ecosystem upgrade plan
═══════════════════════════════════════════════════════════

── HORUS CORE ─────────────────────────────────────────────
  ↑ horus CLI             0.1.9 → 0.2.0
  ↑ horus_core            0.1.9 → 0.2.0 (rebuild)
  ↑ horus_library         0.1.9 → 0.2.0 (rebuild)
  ↑ horus_macros          0.1.9 → 0.2.0 (rebuild)
  ↑ horus_py              0.1.9 → 0.2.0 (maturin rebuild + pip reinstall)
  · horus-sim3d           1.0.0 (already latest)

── RUST TOOLCHAIN ─────────────────────────────────────────
  ↑ rustup                1.28.0 → 1.28.1
  ↑ rustc                 1.82.0 → 1.83.0 (stable channel)
  · clippy                (bundled with rustc)
  · rustfmt               (bundled with rustc)

── PYTHON TOOLS ───────────────────────────────────────────
  ↑ ruff                  0.8.2 → 0.9.0
  · pytest                8.3.0 (already latest)
  + pytest-cov            (not installed — add with --all)
  + cargo-audit           (not installed — add with --all)

── PLUGINS ────────────────────────────────────────────────
  ↑ horus-ros-bridge      0.9.0 → 0.9.3 (fixes horus 0.2.0 compat)
  · horus-gazebo          1.0.0 (already latest)
  · my-custom-tool        0.2.0 (local — skipped)

── PROJECT DEPENDENCIES (my-robot) ────────────────────────
  ↑ serde                 1.0.217 → 1.0.219
  ↑ tokio                 1.42.0 → 1.43.1
  · rapier3d              0.22.0 (pinned — skipped)
  ↑ numpy                 1.26.4 → 2.1.0 (major — confirm?)
  · opencv-python         4.9.0 (already latest)

── SYSTEM ─────────────────────────────────────────────────
  · RT config             up to date
  ! stale SHM             2 namespaces will be cleaned

═══════════════════════════════════════════════════════════
  7 upgrades, 2 new installs, 1 major version bump
  Proceed? [Y/n/select]
═══════════════════════════════════════════════════════════
```

**Upgrade phases** (executed in order, each can be skipped with flags):

```
Phase 1: HORUS CORE  (--cli)
  1a. Check latest horus version from GitHub releases / registry API
  1b. If source install (~/softmata/horus exists):
      → git pull origin main
      → cargo build --release
      → ./install.sh (rebuilds + installs CLI, core, library, macros)
  1c. If binary install:
      → Download pre-built binary for platform
      → Replace ~/.cargo/bin/horus (or /usr/local/bin/horus)
  1d. Update ~/.horus/installed_version
  1e. Rebuild horus_py if installed:
      → cd horus_py && maturin build --release
      → pip install --force-reinstall target/wheels/*.whl
  1f. Rebuild horus-sim3d if installed:
      → cd ../horus-sim3d && cargo build --release --no-default-features

Phase 2: PLUGINS  (--plugins)
  4a. For each installed plugin:
      → Check registry for newer version
      → Skip local/path plugins
      → Check version compatibility with new CLI version
      → Download + verify + install update
  4b. Re-verify all plugins after upgrade

Phase 3: PROJECT DEPS  (--deps)
  5a. Rust deps: regenerate .horus/Cargo.toml, cargo update
  5b. Python deps: pip install --upgrade (scoped to .horus/packages/)
  5c. Registry deps: check for newer versions, download if available
  5d. Regenerate horus.lock
  5e. Run horus check to validate

Phase 4: CLEANUP
  4a. Clean stale SHM namespaces
  4b. Remove orphaned cache entries
  6c. Clean stale workspace registry entries
  6d. Rebuild .horus/Cargo.toml from horus.toml (if schema changed)
```

**Safety features:**

- **Major version bumps require confirmation**: `numpy 1.26 → 2.1` prompts user
- **Pinned deps are skipped**: `rapier3d = "0.22"` (exact pin) is never auto-upgraded
- **Dry-run by default for deps**: Shows plan, waits for confirmation
- **Rollback on failure**: If any phase fails, rolls back that phase's changes
- **Local plugins skipped**: Path-based plugins are the user's responsibility
- **Config migration**: If horus.toml schema changed between versions, auto-migrate

**Implementation:** `commands/upgrade.rs`

```rust
pub struct UpgradeConfig {
    pub check_only: bool,
    pub yes: bool,               // skip confirmation
    pub version: Option<String>,  // target version
    // Scope flags (all true if none specified)
    pub cli: bool,               // horus CLI + core libs + horus_py
    pub plugins: bool,           // installed plugins
    pub deps: bool,              // project dependencies
}

pub fn run_upgrade(cfg: UpgradeConfig) -> Result<()> {
    let ctx = dispatch::detect_context()?;
    let mut plan = UpgradePlan::new();

    // ── Phase 1: HORUS CORE ──
    if cfg.cli || cfg.is_everything() {
        plan_horus_core_upgrade(&ctx, &mut plan, cfg.version.as_deref())?;
    }

    // ── Phase 2: PLUGINS ──
    if cfg.plugins || cfg.is_everything() {
        plan_plugins_upgrade(&ctx, &mut plan)?;
    }

    // ── Phase 3: PROJECT DEPS ──
    if cfg.deps || cfg.is_everything() {
        if let Some(ref manifest) = ctx.manifest {
            plan_deps_upgrade(&ctx, manifest, &mut plan)?;
        }
    }

    // ── Show plan ──
    plan.print();

    if plan.is_empty() {
        cli_output::success("Everything is up to date");
        return Ok(());
    }

    if cfg.check_only { return Ok(()); }

    // ── Confirm ──
    if !cfg.yes {
        let choice = prompt_upgrade_confirmation(&plan)?;
        match choice {
            Confirm::Yes => {},
            Confirm::Select => { plan.interactive_select()?; },
            Confirm::No => { cli_output::info("Cancelled"); return Ok(()); },
        }
    }

    // ── Execute ──
    plan.execute_with_rollback()?;

    // ── Phase 6: CLEANUP ──
    let stale = list_stale_namespaces()?;
    if !stale.is_empty() {
        clean_shared_memory(false, false)?;
        cli_output::info(&format!("Cleaned {} stale SHM namespaces", stale.len()));
    }
    clean_stale_workspaces()?;
    clean_orphaned_cache()?;

    cli_output::success("Upgrade complete");
    Ok(())
}

fn plan_horus_core_upgrade(ctx: &ProjectContext, plan: &mut UpgradePlan, target: Option<&str>) -> Result<()> {
    let current = semver::Version::parse(env!("CARGO_PKG_VERSION"))?;

    // Check for source install vs binary install
    let source_dir = find_horus_source_dir();

    let latest = match target {
        Some(v) => semver::Version::parse(v)?,
        None => fetch_latest_horus_version()?,
    };

    if latest <= current {
        plan.skip("horus CLI", &current.to_string(), "already latest");
        return Ok(());
    }

    plan.upgrade("horus CLI", &current.to_string(), &latest.to_string());
    plan.upgrade("horus_core", &current.to_string(), &latest.to_string());
    plan.upgrade("horus_library", &current.to_string(), &latest.to_string());
    plan.upgrade("horus_macros", &current.to_string(), &latest.to_string());

    if source_dir.is_some() {
        plan.set_strategy("horus", UpgradeStrategy::SourceBuild {
            dir: source_dir.unwrap(),
            steps: vec![
                "git pull origin main",
                "cargo build --release",
                "./install.sh",
            ],
        });
    } else {
        plan.set_strategy("horus", UpgradeStrategy::BinaryDownload {
            url: format!("https://github.com/softmata/horus/releases/download/v{}/horus-{}-{}",
                latest, std::env::consts::OS, std::env::consts::ARCH),
        });
    }

    // horus_py rebuild
    if ctx.tools.python.is_some() {
        if let Ok(py_ver) = get_installed_horus_py_version(ctx) {
            plan.upgrade("horus_py", &py_ver, &latest.to_string());
            plan.set_strategy("horus_py", UpgradeStrategy::MaturinRebuild);
        }
    }

    // horus-sim3d rebuild
    let sim3d_dir = find_horus_sim3d_dir();
    if let Some(dir) = sim3d_dir {
        let sim_ver = read_cargo_version(&dir.join("Cargo.toml"))?;
        plan.upgrade("horus-sim3d", &sim_ver, "(rebuild with new core)");
        plan.set_strategy("horus-sim3d", UpgradeStrategy::SourceBuild {
            dir,
            steps: vec!["cargo build --release --no-default-features"],
        });
    }

    Ok(())
}

fn plan_deps_upgrade(ctx: &ProjectContext, manifest: &HorusManifest, plan: &mut UpgradePlan) -> Result<()> {
    for (name, dep) in &manifest.dependencies {
        let current_ver = dep.version().unwrap_or("*");

        // Skip exact pins (user intentionally pinned)
        if !current_ver.contains('^') && !current_ver.contains('~')
            && !current_ver.contains('*') && !current_ver.contains('>')
            && current_ver != "*" {
            // Exact pin like "0.22" — skip
            plan.skip(name, current_ver, "pinned");
            continue;
        }

        match dep.source() {
            Some("crates.io") | None => {
                if let Ok(latest) = check_crates_io_latest(name) {
                    if latest != current_ver {
                        let is_major = is_major_bump(current_ver, &latest);
                        if is_major {
                            plan.major_upgrade(name, current_ver, &latest);
                        } else {
                            plan.upgrade(name, current_ver, &latest);
                        }
                    }
                }
            }
            Some("pypi") => {
                if let Ok(latest) = check_pypi_latest(name) {
                    if latest != current_ver {
                        let is_major = is_major_bump(current_ver, &latest);
                        if is_major {
                            plan.major_upgrade(name, current_ver, &latest);
                        } else {
                            plan.upgrade(name, current_ver, &latest);
                        }
                    }
                }
            }
            Some("registry") => {
                if let Ok(latest) = check_registry_latest(name) {
                    if latest != current_ver {
                        plan.upgrade(name, current_ver, &latest);
                    }
                }
            }
            _ => {} // path, git — user-managed
        }
    }

    Ok(())
}
```

**Version detection helpers:**

```rust
/// Find horus source directory (for source-based upgrades).
/// Checks: $HORUS_SOURCE_DIR, ~/softmata/horus, ../horus (relative to binary)
fn find_horus_source_dir() -> Option<PathBuf> {
    if let Ok(dir) = std::env::var("HORUS_SOURCE_DIR") {
        let p = PathBuf::from(dir);
        if p.join("Cargo.toml").exists() { return Some(p); }
    }
    let home = dirs::home_dir()?;
    let candidates = [
        home.join("softmata/horus"),
        home.join("horus"),
    ];
    candidates.iter().find(|p| p.join("Cargo.toml").exists()).cloned()
}

/// Find horus-sim3d source directory.
fn find_horus_sim3d_dir() -> Option<PathBuf> {
    let home = dirs::home_dir()?;
    let candidates = [
        home.join("softmata/horus-sim3d"),
        home.join("horus-sim3d"),
    ];
    candidates.iter().find(|p| p.join("Cargo.toml").exists()).cloned()
}
```

---

## `[scripts]` Section — Project-Specific Commands

Allow users to define custom commands in `horus.toml`:

```toml
[scripts]
sim = "horus run sim_node.rs -- --world warehouse"
calibrate = "horus run calibration.py -- --camera front"
deploy-staging = "horus deploy --target staging.local --release"
test-hw = "horus test --integration -- --hardware"
lint-all = "horus fmt --check && horus lint --strict"
```

**Usage:**

```bash
horus run sim                   # runs the "sim" script
horus run calibrate             # runs the "calibrate" script
horus run deploy-staging        # runs the "deploy-staging" script
```

**Resolution order** (in `commands/run/mod.rs`):
1. Is input a file? → execute file
2. Is input a glob pattern? → expand and execute
3. Is input a script name from `horus.toml [scripts]`? → execute script
4. Is input a directory? → auto-detect main file
5. None of the above → error with suggestions

**Implementation:**
```rust
fn resolve_execution_target(input: &str, manifest: &Option<HorusManifest>) -> ExecutionTarget {
    // Check files first
    let path = Path::new(input);
    if path.exists() && path.is_file() {
        return ExecutionTarget::File(path.to_path_buf());
    }

    // Check scripts
    if let Some(ref m) = manifest {
        if let Some(script_cmd) = m.scripts.get(input) {
            return ExecutionTarget::Script(script_cmd.clone());
        }
    }

    // Check glob, directory, etc.
    // ...
}
```

---

## Unified Lockfile: `horus.lock`

Instead of `.horus/Cargo.lock` with gitignore exceptions, generate a single `horus.lock` at project root:

```toml
# AUTO-GENERATED by horus — do not edit manually.
# Contains pinned versions for reproducible builds across all languages.
# Commit this file to version control.

[metadata]
generated_by = "horus 0.5.0"
generated_at = "2026-03-11T14:30:00Z"
horus_toml_hash = "sha256:abc123..."

# Rust dependencies (from .horus/Cargo.lock)
[rust]
serde = { version = "1.0.217", checksum = "sha256:..." }
tokio = { version = "1.42.0", checksum = "sha256:..." }
# ... all transitive deps

# Python dependencies (from pip freeze)
[python]
numpy = { version = "1.26.4", hash = "sha256:..." }
opencv-python = { version = "4.9.0.80", hash = "sha256:..." }

# Horus registry packages
[registry]
my-horus-sensor = { version = "2.1.0", checksum = "sha256:..." }
```

**How it works:**
1. After every `horus build` or `horus install`, regenerate `horus.lock`
2. Read `.horus/Cargo.lock` for Rust pins
3. Run `pip freeze` (scoped to `.horus/packages/`) for Python pins
4. Read `.horus/packages/*/metadata.json` for registry pins
5. Write combined `horus.lock` at project root
6. `.gitignore` ignores `.horus/` entirely (no exceptions needed)
7. `horus build` reads `horus.lock` to verify consistency — warns if deps changed without lock update

**On `horus env restore`:**
```bash
horus env restore horus.lock
# 1. Parse horus.lock
# 2. Write horus.toml [dependencies] from lock entries
# 3. Generate .horus/Cargo.lock from [rust] section
# 4. pip install exact versions from [python] section
# 5. Download registry packages from [registry] section
# 6. Run horus build to verify
```

---

## Updated Implementation Order

### Phase 0: Infrastructure (foundation for everything)
1. Create `src/dispatch.rs` — `ProjectContext`, `ToolChain`, `detect_context()`, `detect_toolchain()`
2. Create `src/run_with_prefix.rs` — prefixed parallel output utility

### Phase 1-7: (unchanged from original document)

### Phase 8: Smart Commands
23. Create `commands/fmt.rs` — formatting dispatch
24. Create `commands/lint.rs` — linting dispatch
25. Create `commands/doc.rs` — documentation dispatch
26. Create `commands/bench.rs` — benchmark dispatch
27. Extend `commands/test.rs` — add Python test dispatch
28. Create `commands/doctor.rs` — toolchain health check
29. Create `commands/shell.rs` — activated subshell
30. Create `commands/migrate.rs` — project migration
31. Create `commands/deps.rs` — dependency insight subcommands
32. Create `commands/upgrade.rs` — CLI self-update
33. Add `[scripts]` to `HorusManifest` + resolution in `commands/run/mod.rs`
34. Create `src/lockfile_unified.rs` — unified `horus.lock` generation

### Phase 9: Registration
36. Register all new commands in `main.rs` `Commands` enum
37. Update shell completions
38. Update `horus --help` grouping

---

## Updated Files Affected

| File | Change Type | Description |
|------|------------|-------------|
| `src/dispatch.rs` | NEW | Smart dispatch core: ProjectContext, ToolChain, detect |
| `commands/fmt.rs` | NEW | Format dispatch: cargo fmt + ruff/black |
| `commands/lint.rs` | NEW | Lint dispatch: cargo clippy + ruff |
| `commands/doc.rs` | NEW | Doc dispatch: cargo doc + pdoc/sphinx |
| `commands/bench.rs` | NEW | Bench dispatch: cargo bench + pytest-benchmark |
| `commands/doctor.rs` | NEW | Toolchain health check |
| `commands/shell.rs` | NEW | Activated subshell with cargo/pip aliases |
| `commands/migrate.rs` | NEW | Project migration from native build files |
| `commands/deps.rs` | NEW | Dependency insight: tree, why, outdated, audit |
| `commands/upgrade.rs` | NEW | CLI self-update |
| `commands/test.rs` | MODIFY | Add Python pytest dispatch |
| `commands/run/mod.rs` | MODIFY | Add `[scripts]` resolution |
| `manifest.rs` | MODIFY | Add `scripts`, `dev_dependencies` fields |
| `src/lockfile_unified.rs` | NEW | Unified horus.lock generation |
| `main.rs` | MODIFY | Register 8 new commands |

---

## Complete Command Coverage

Every tool the user would reach for is covered by a horus equivalent:

| User wants to | Native tool | horus equivalent | Dispatch |
|---|---|---|---|
| Format Rust | `cargo fmt` | `horus fmt` | auto |
| Format Python | `ruff format` / `black` | `horus fmt` | auto |
| Lint Rust | `cargo clippy` | `horus lint` | auto |
| Lint Python | `ruff check` / `pylint` | `horus lint` | auto |
| Type check Python | `mypy` / `pyright` | `horus lint --types` | auto |
| Test Rust | `cargo test` | `horus test` | auto |
| Test Python | `pytest` | `horus test` | auto |
| Coverage | `cargo-tarpaulin` / `pytest-cov` | `horus test --coverage` | auto |
| Build Rust | `cargo build` | `horus build` | auto |
| Run Python | `python main.py` | `horus run` | auto |
| Docs Rust | `cargo doc` | `horus doc` | auto |
| Docs Python | `pdoc` / `sphinx` | `horus doc` | auto |
| Bench Rust | `cargo bench` | `horus bench` | auto |
| Bench Python | `pytest-benchmark` | `horus bench` | auto |
| Add dep (Rust) | `cargo add serde` | `horus install serde` | auto-detect crates.io |
| Add dep (Python) | `pip install numpy` | `horus install numpy` | auto-detect pypi |
| Add dep (registry) | — | `horus install my-pkg` | auto-detect registry |
| Remove dep | `cargo remove` / `pip uninstall` | `horus remove` | auto |
| Update deps | `cargo update` | `horus update` | auto |
| Dep tree | `cargo tree` | `horus deps tree` | auto |
| Dep audit | `cargo audit` | `horus deps audit` | auto |
| Outdated deps | `cargo outdated` | `horus deps outdated` | auto |
| Clean | `cargo clean` | `horus clean` | auto |
| Check | `cargo check` | `horus check` | auto |
| Publish | `cargo publish` | `horus publish` | auto |
| Deploy | `scp` / `rsync` | `horus deploy` | — |
| Freeze env | `pip freeze` / lockfile | `horus env freeze` | unified |
| Restore env | — | `horus env restore` | unified |
| Health check | — | `horus doctor` | — |
| Subshell | — | `horus shell` | — |
| Custom scripts | `Makefile` / `just` | `horus run <script>` | from horus.toml |
| Self-update | — | `horus upgrade` | — |
| Migrate project | — | `horus migrate` | — |
| Detect hardware | — | `horus drivers detect` | registry API |
