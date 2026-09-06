# Architecture — where a change goes

This file answers one question: you have cloned the repository, you know what you
want to change, and you need to know which crate, which module and which file to
open. It is a map, not a tutorial. It does not explain how the scheduler works,
only where the scheduler lives and what else you have to touch when you change
it.

Before this file existed, the whole answer was two directory listings: sixteen
lines at `CONTRIBUTING.md:252-267` and fifteen at `README.md:626-640`. Both stop
at the crate boundary. Neither tells you that adding a framework message type
means writing it into six places, or that `horus_cpp_macros` — the crate whose
name says "C++ binding codegen" — is a prototype no crate in the workspace
depends on. Those are the facts that cost a newcomer a day.

**What this file is not.** It is not a design document; it says where code is,
not why the design is what it is. It does not cover the build system, CI gates,
the test tree, the shared-memory wire format, or the `horus_net` protocol —
those have their own documents, and where one does not exist yet this file says
so rather than sketching it. Every claim below cites the file and line it was
read from. Where the code contradicts its own comments, this file says which one
is true and leaves the contradiction visible rather than picking the tidier
sentence.

## Reading this file

Sizes are measured, not estimated. Reproduce them with:

```sh
find . -name '*.rs' -not -path './target/*' -not -path './*/target/*' \
       -not -path './.git/*' -not -path './.claude/*' -not -path '*/.horus/*' \
     | xargs wc -l | tail -1
```

which gives **441,900** lines of Rust: 321,877 under `*/src`, 114,496 under
`*/tests`. That is the tree you are navigating.

Every grep in this file is printed beside its number. Re-run them rather than
trusting them; they were taken on the tree at the time of writing.

## The workspace at a glance

Eleven members, declared at `Cargo.toml:3-15`. Line counts are `*/src` only, and
sum to the 321,877 above.

| Crate | src lines | One-line job |
|---|---:|---|
| `horus` | 1,252 | Umbrella facade: the prelude and the `net` feature switch. |
| `horus_core` | 112,344 | The runtime. Nodes, topics, scheduler, memory, services, actions, drivers. |
| `horus_macros` | 800 | Proc macros for Rust users: `node!`, `LogSummary`. |
| `horus_cpp_macros` | 2,731 | Prototype. No crate depends on it. See below. |
| `horus_cpp` | 7,461 | C ABI surface plus hand-written C++17 headers. |
| `horus_manager` | 131,514 | The `horus` CLI, and the largest crate in the tree. |
| `horus_types` | 4,621 | Universal POD IPC types: math, diagnostics, time, generic. |
| `horus_net` | 15,231 | LAN replication over UDP. |
| `horus_sys` | 12,827 | Platform abstraction layer. |
| `benchmarks` | 14,031 | Performance suite (package `horus_benchmarks`). |
| `horus_py` | 19,065 | PyO3 bindings plus the hand-written Python half. |

Two dependencies that look like members and are not: `horus-robotics` and
`horus-tf` are pinned git dependencies (`Cargo.toml:40`, `horus/Cargo.toml:25`),
redirected at the local tree by the two `[patch]` tables at `Cargo.toml:108-115`.
Standard robotics messages (`CmdVel`, `Imu`, `LaserScan`) and the transform tree
live there, not here, and both reach the user through `horus::prelude`
(`horus/src/lib.rs:613`, `:631`). A change to `CmdVel` is a change to a different
repository.

The two patch tables are not symmetric, and the asymmetry is load-bearing:
`horus-robotics` gets `horus_core`, `horus_types` and `horus_macros`
(`Cargo.toml:108-111`); `horus-tf` gets only `horus_core` and `horus_macros`
(`:113-115`). Copy the wrong one and Cargo will resolve `horus_types` from the
git rev instead of your tree.

The root `tests/` directory is not a Rust crate and not a workspace member: it
holds shell scripts (`cli_runtime_test.sh`, `test_horus_cli.sh`,
`test_horus_integration.sh`, `valgrind_dlpack.sh`), `docker/` and `qa/`. Rust
tests live in each crate's own `tests/`.

Every member inherits `rust-version = "1.90"` (`Cargo.toml:28`) and the workspace
lint set (`Cargo.toml:83-102`), and all eleven do so today — each manifest
carries both `rust-version.workspace = true` and `[lints] workspace = true`
(checked per crate). A new member must opt into both explicitly or it inherits
neither; nine crates once declared no floor at all and `horus_sys` declared 1.75,
which is why the comment at `Cargo.toml:24-27` exists.

---

## horus

The user-facing facade, and almost nothing else — 1,252 lines across
`src/lib.rs` (868) and `src/time.rs` (384).

**Belongs here:** the prelude (`src/lib.rs:581`), which is the curated list of
names a user gets from `use horus::prelude::*`; the `net` feature and its ctor
auto-wire hook (`:213-225`); thin re-export modules — `types` (`:445`), `msg`
(`:519`), `hardware` (`:371`).

**Does not belong here:** implementation. Every type in the prelude is
re-exported from somewhere else — `horus_core`, `horus_types`, `horus_robotics`,
`horus_tf`. If you are adding a function body to this crate, you are probably in
the wrong crate.

Adding a user-facing type means two edits, not one: define it where it belongs,
then add it to the prelude at `horus/src/lib.rs:581` or it is unreachable by the
name users expect.

## horus_core

The runtime. 112,344 lines, the module list is twelve `pub mod` declarations in a
96-line `src/lib.rs`, and every one of them is `#[doc(hidden)]` except
`terminal`. Its own header says so: "This is an internal crate. Users should
depend on `horus`" (`horus_core/src/lib.rs:15-16`). See
[The public API boundary](#the-public-api-boundary) for how true that is.

**Belongs here:** anything that runs inside a robot process — node lifecycle,
IPC, scheduling, memory, safety.

**Does not belong here:** anything the CLI does (that is `horus_manager`),
anything that touches a platform API directly (that is `horus_sys`, mostly — see
the layering section), and anything only one language binding needs.

Module by module, from `horus_core/src/lib.rs:20-52`:

| Module | Owns |
|---|---|
| `actions` | Long-running goals with feedback and cancellation. Client, server, goal handles (`actions/mod.rs:1-9`). |
| `communication` | `Topic<T>` and everything under it: SHM header and layout, dispatch, pools, fan-out, seqlock, migration (`communication/mod.rs:1-9`, and the file list under `communication/topic/`). |
| `core` | `Node`, node presence and metrics, clocks, `Rate`/`Stopwatch`, RT config, `hlog!` plumbing, tick context. |
| `drivers` | `[hardware]` loading from `horus.toml`: the node registry, `NodeParams`, and `ExecDriver` (`drivers/mod.rs:1-7`). |
| `error` | `HorusError` and its domain sub-errors; the whole error taxonomy (`error.rs:1-8`). |
| `memory` | Shared regions, tensor pool and handles, the pool backend trait, image/pointcloud/depth/costmap/grid domain types, the RT allocator, SIMD helpers. |
| `params` | `RuntimeParams` — the runtime key/value store (`params.rs:1-3`). Distinct from `drivers::NodeParams`, which is the `horus.toml` view. |
| `scheduling` | The scheduler, all five execution paths, node builder, dependency graph, safety monitor, fault tolerance, blackbox, profiler, record/replay, telemetry. |
| `services` | Blocking request/response RPC (`services/mod.rs:1-8`). |
| `terminal` | Non-panicking console output. **The only module not `doc(hidden)`** — because `println!` panics when the write fails and a robot's stdout fails routinely (`lib.rs:38-44`, `terminal.rs:1-9`). |
| `types` | Tensor dtypes, device, and the zero-copy descriptors (image, pointcloud, depth, costmap, occupancy grid). |
| `testing` | `MockTopic`, `TestNode`, `ShmFaultInjector`. Compiled only under `cfg(test)` or the `test-utils` feature (`lib.rs:50-52`). |

`utils` is `pub(crate)` (`lib.rs:47`) and is not part of the surface.

One correction worth carrying: `communication/topic/dispatch.rs:90-92` states as a
safety invariant that `Topic<T>` is "`!Send + !Sync`". It is `Send` —
`unsafe impl<T: TopicMessage> Send for Topic<T> where T::Wire: Send` at
`communication/topic/mod.rs:3711` — and not `Sync`, which the module header
states correctly at `:36-48`. Sharing one handle needs a `Mutex`, not an
`RwLock`; a read guard hands out concurrent `&Topic` and that is exactly the race
the `Sync` impl was removed to prevent (`:42-48`).

## horus_macros

800 lines. Two macros for Rust users: `node!` (`#[proc_macro]` at
`horus_macros/src/lib.rs:76`) and the `LogSummary` derive (`:102`), both named in
the crate header at `:10-11`. It is an optional dependency of `horus`, behind the
`macros` feature, which is on by default (`horus/Cargo.toml:18`, `:39`, `:46`).

**Belongs here:** proc macros that expand into user code.

**Does not belong here:** the `message!` macro, which is `macro_rules!` and lives
at `horus_core/src/communication/macros.rs:120-121`.

## horus_cpp_macros

2,731 lines that nothing consumes. Its own header is unambiguous: "**Status:
unwired prototype. Nothing in HORUS applies `#[horus_api]`**"
(`horus_cpp_macros/src/lib.rs:3-14`), and two independent checks agree with it:

- Every application of the attribute (`grep -rn '#\[horus_api'`) is in this
  crate's own trybuild fixtures under `horus_cpp_macros/tests/pass/` and
  `tests/fail/`. Nothing else in the tree applies it.
- No member manifest lists `horus_cpp_macros` as a dependency. It is declared in
  `[workspace.dependencies]` at `Cargo.toml:35` and consumed by nobody; the
  `horus_cpp` edge that once existed has been removed (`src/lib.rs:6-7`).

The crate's own header claims the only `horus_api` hits are inside its fixtures
(`:5-6`). That is true of applications, not of the string: the name also appears
in `Cargo.toml:7`, `CONTRIBUTING.md:261`, `horus_cpp_macros/Cargo.toml:8` and as
a test function name at `horus_manager/tests/docs_scaffold.rs:581`. Nothing
turns on it, but do not use that grep as your check.

**Do not put a C++ binding here.** The crate reads like the C++ codegen path and
is not it. Read the `horus_cpp` section instead.

## horus_cpp

The real C++ path. 7,461 lines: the `extern "C"` surface in `src/*_ffi.rs`
(`action`, `node`, `params`, `pool`, `scheduler`, `service`, `topic`,
`transform`, `types`) plus `src/c_api.rs`, the layout contract, three test
modules, and hand-written C++ headers in `include/horus/`.

There is **no `#[cxx::bridge]`**; the `cxx` (`Cargo.toml:22`) and `cxx-build`
(`:29`) dependencies are placeholders (`horus_cpp/src/lib.rs:5-9`). `build.rs` is
eleven lines: an empty `fn main()` whose entire body is comments, including the
`cxx_build::bridge("src/scheduler_ffi.rs")` call the crate doc describes as
existing — it is commented out, not wired (`horus_cpp/build.rs:1-11`).
`CONTRIBUTING.md:260` calls this crate "CXX"; that is not what it is today.

The headers are C++17 in practice: `include/horus/impl/scheduler_impl.hpp:263`
relies on C++17 inline variables, and the contract test compiles the generated
header with `-std=c++17` (`src/layout_contract.rs:254`).

**Belongs here:** FFI wrappers, the C ABI, the `.hpp` headers, and the layout
contract.

The layout contract is the load-bearing part. `src/layout_contract.rs` renders
`include/horus/layout_contract.hpp` from the Rust types with `offset_of!`, and
`contract_file_is_current` (`:214`) fails when the committed header has drifted
(`:30-33`). It exists because `JointCommand` was once 928 bytes in Rust against
88 in C++ — an 840-byte overrun on every receive — and because `ServoCommand` was
24 bytes on *both* sides while meaning different fields, which no `sizeof` sweep
can catch (`:10-26`). Regenerate it deliberately:

```sh
cargo test -p horus_cpp --no-default-features regenerate_layout_contract -- --ignored --nocapture
```

`regenerate_layout_contract` is a `#[test]` marked `#[ignore]` at `:200-207`, so
`--ignored` is not optional. The command is written out in three places —
the module doc (`:38`), a comment above the type list (`:169`), and the failure
message `contract_file_is_current` prints (`:221`, which omits `--nocapture`).

## horus_manager

The `horus` CLI, 131,514 lines — larger than the runtime it drives.

It is one package with two binaries and a library: `[lib] horus_manager`
(`horus_manager/Cargo.toml:22-24`), `[[bin]] horus` from `src/main.rs`
(`:26-28`), and `[[bin]] ci_test_node` (`:30-32`). The split matters: the clap
`Commands` enum is declared `enum Commands` — private — at `src/main.rs:133-1174`,
in the *binary*, and the name appears nowhere in `src/lib.rs`. Integration tests
cannot reach it as a type at all. They shell out instead, which is why
`tests/help_contract.rs` parses rendered `--help` output rather than the source
string (`help_contract.rs:29-31`).

**Belongs here:** subcommands (`src/commands/`, 43 `pub mod` lines in a 44-line
`commands/mod.rs`), project generation (`cargo_gen.rs`, `cmake_gen.rs`,
`pyproject_gen.rs`), the manifest model (`manifest.rs`, `manifest_lint.rs`),
message-spec codegen (`msgspec/`), live introspection (`discovery/`), the package
registry client (`registry/`), and the plugin system (`plugins/`).

**Does not belong here:** anything a running robot needs. This crate is a build
and operations tool; nothing in a node's tick path should reach it.

## horus_types

4,621 lines of `#[repr(C)]` POD messages shared by every HORUS vertical: math,
diagnostics, time, generic (`horus_types/src/lib.rs:1-6`).

POD types are registered through `impl_pod_message!` (`horus_types/src/lib.rs:37`),
invoked in exactly three files: `math.rs:792`, `diagnostics.rs:683` and
`time.rs:198`. `generic.rs` is the exception — it holds `GenericMessage` (`:62`),
a serde/MessagePack payload type, and registers nothing through the macro.

The macro takes the field list in full and checks, at compile time, that every
named field exists with the written type, sits at the `repr(C)` offset the list
implies, and that the struct is the size the list implies (`lib.rs:28-36`).
Rename, retype, reorder, add or remove a field without updating the list and the
crate does not build. That strictness is the fix for `Twist` published from C++
and read from Rust with `linear` and `angular` swapped, which two builds once
agreed to share without complaint because HORUS's own shipped types carried no
layout hash at all (`lib.rs:11-22`).

## horus_net

15,231 lines. Transparent LAN replication of SHM topics over UDP — same
`Topic<T>` API, no config (`horus_net/src/lib.rs:1-12`). Opt-in: the `net`
feature on `horus` (`horus/Cargo.toml:44`).

Read `horus_net/src/lib.rs:14-44` before you change anything in this crate. The
data path is unauthenticated by design and the header says so at length: any
host that can send UDP to the port and pass the source-address filter can write
arbitrary bytes into local SHM topics, including actuation commands
(`:16-25`). `HORUS_NET_SECRET` is a cleartext FNV-1a value in every announcement
and stops accidental cross-fleet mixing, not an attacker (`:26-29`). Only the
e-stop channel is authenticated, by HMAC keyed on `HORUS_ESTOP_KEY`, and those
packets are rejected outright when no key is provisioned (`:35-37`). A change
that widens what the network can reach is a security change.

The protocol document these modules cite does not exist in this repository.
Twelve source files carry a `blueprint section N` reference — `wire.rs:4`,
`encoding.rs:3`, `priority.rs:3`, `optimize/mod.rs:4`, `discovery.rs:5`,
`metrics.rs:6`, `flow_control.rs:6`, `heartbeat.rs:6`, `reliability.rs:7`,
`fragment.rs:7`, `guard.rs:8`, `replicator.rs:9` — as does
`tests/phase1_validation.rs:2`, and `tests/safety_tests.rs:150` is a test named
`config_defaults_match_blueprint` that pins `NetConfig` defaults
(`heartbeat_ms: 50`, `missed_threshold: 3`, `on_link_lost: "warn"`) against it.
`find . -iname '*blueprint*'` outside `target/` and `.git/` returns nothing. Do
not assume a spec you cannot open.

## horus_sys

12,827 lines. The platform abstraction layer, and the only crate whose header
makes a claim about the rest of the tree: "All `#[cfg(target_os)]` code lives
here — horus_core and horus_manager import from horus_sys and never use platform
APIs directly" (`horus_sys/src/lib.rs:4-5`). That claim is not true today. See
[The horus_sys layering rule](#the-horus_sys-layering-rule).

Ten modules: `device`, `discover`, `fs`, `platform`, `process`, `rt`, `shm`,
`sync`, `terminal`, `time` (`horus_sys/src/lib.rs:9-20`, `:29-38`).

The header says each module "has per-platform backends selected at compile time"
(`:24`). The directory does not bear that out, and it matters when you are
copying a pattern:

- **Split into per-platform files:** `shm` (`linux.rs`, `macos.rs`,
  `windows.rs`, `fallback.rs`), `rt` (three, plus `pi_mutex.rs`), `platform`
  (three), `discover` (three). Only `shm` has a `fallback.rs`.
- **One `mod.rs` with `cfg` inside it:** `process`, `fs`, `time`, `terminal`.
- **No platform branching in the module root at all:** `device/mod.rs` is six
  lines of doc comment; `sync/` splits by concern (`cpp.rs`, `python.rs`,
  `rust.rs`, `system.rs`, `lockfile.rs`), not by OS.

`shm` is the template worth copying: `mod.rs:12-28` does the `cfg` selection and
the re-export in one block — one `#[cfg(target_os = …)] mod` line and one
matching `pub use` per platform, plus a `fallback` arm for everything else.

## benchmarks

14,031 lines, package name `horus_benchmarks` (`benchmarks/Cargo.toml:2`).
Criterion-style benches in `benchmarks/benches/` and standalone binaries in
`benchmarks/src/bin/`, run with
`cargo run --release -p horus_benchmarks --bin <name>`
(`benchmarks/Cargo.toml:19`, `[[bin]]` tables from `:22`). `horus_cpp` and
`horus_sys` also carry their own `benches/`.

This crate does its own platform detection (`src/platform.rs`) and declares
`libc` unconditionally (`benchmarks/Cargo.toml:61`). It is not covered by the
`horus_sys` rule, but it is not exempt from it either — see the census below.

## horus_py

19,065 lines. PyO3 bindings plus the hand-written Python half.

Two halves, and you usually have to touch both:

- Rust: `horus_py/src/*.rs` defines `#[pyclass]` wrappers, registered in the
  `#[pymodule] fn _horus` at `horus_py/src/lib.rs:60-90`.
- Python: `horus_py/horus/` holds `__init__.py`, `_horus.pyi`, `py.typed`,
  `perception.py` and `msggen/` (plus a built `_horus.abi3.so` when the
  extension has been compiled). `Node` never reaches Rust at all; its docstrings
  are the API reference people actually read, which is why the test-only
  `src/python_api_docs.rs` pins them (`horus_py/src/lib.rs:45-50`).

The one trap: `extension-module` is a default feature
(`horus_py/Cargo.toml:51`), so `cargo test -p horus_py` fails to link with
roughly forty `undefined symbol: PyEval_*` lines. Run the tests with
`--no-default-features`. The crate now says this in a `compile_error!`
(`horus_py/src/lib.rs:11-18`) rather than a comment in `Cargo.toml`, because a
comment in `Cargo.toml` is not where anyone is looking when the linker fails
(`:1-10`).

---

## Where does my change go

Each row was checked by finding where the existing ones live. Where the answer is
"edit these named lines" rather than "add a file", the row says so — an
extension point that is not routable is worse than no extension point, because
you find out after you have written the implementation.

| You are adding | It goes in | And you must also |
|---|---|---|
| **A network transport** | `horus_net/src/transport/`, alongside `udp.rs` | Edit `replicator.rs:75`. `Transport` is a public trait (`transport/mod.rs:9-25`) and `UdpTransport` (`transport/udp.rs:60`) is its only real implementation — the other five `impl Transport for` sites are mocks in test modules and in the always-compiled `heartbeat::tests_support` (`heartbeat.rs:321-349`). `Replicator` holds a concrete `transport: UdpTransport` field, bound at `replicator.rs:131`. There is no routing by name or config. |
| **A topic data-plane backend** | `horus_core/src/communication/topic/backend.rs` and `dispatch.rs` | Add a `BackendStorage` variant — the enum is `pub(crate)` and closed (`backend.rs:18-29`) — plus a complete send and recv function per path. The existing set is tabulated at `dispatch.rs:65-83`. This is not a user extension point. |
| **A scheduler execution class** | `horus_core/src/scheduling/` | Five edits: a variant on `ExecutionClass` (`types.rs:279-291`), a new executor module beside `rt_executor.rs` / `compute_executor.rs` / `event_executor.rs` / `async_executor.rs`, a selector on `NodeBuilder` (`node_builder.rs:220-266`), an arm in `group_nodes_by_class` (`types.rs:349-373`) plus its `NodeGroups` field, and a start block (`scheduler/mod.rs:3074-3169`). |
| **A scheduler *policy*** | depends which | Failure handling: `FailurePolicy` at `scheduling/fault_tolerance/failure_policy.rs:29`. Budget overrun: `BudgetPolicy` at `scheduling/safety_monitor.rs:238`. Safety state: `SafetyState` at `safety_monitor.rs:221`. Stale data: `StalePolicy` at `scheduling/types.rs:745`. Deadline miss action: `Miss` at `core/rt_node.rs:20`. |
| **A CLI subcommand** | `horus_manager/src/commands/<name>.rs` | Four more edits: `pub mod` in `commands/mod.rs`; a variant on `Commands` (`main.rs:133-1174`); a match arm in `run_command` (`main.rs:2446` onward); and a row in the hand-written `help_template` (`main.rs:17-99`). The template is guarded against the enum by `horus_manager/tests/help_contract.rs`, which exists because `enable`, `disable` and `verify` were advertised as top-level commands and all three errored, `frame`'s alias was listed as `frames` (which errors; the real one is `tf`), and `completion`, `scripts` and `setup-rt` all worked and appeared nowhere (`help_contract.rs:7-15`). |
| **A framework message type** | `horus_types/src/{math,diagnostics,time}.rs` | Write it into the six places `horus_manager/src/msgspec/mod.rs:3-8` names: the `#[repr(C)]` struct and `impl_pod_message!` in `horus_types`; a `#[pyclass]` and a `pod_topic_types!` row in `horus_py` (`horus_py/src/topic.rs:408`); `impl_topic_ffi!` (`horus_cpp/src/topic_ffi.rs:131`) and `impl_pod_topic_c_api!` (`horus_cpp/src/c_api.rs:683`) plus a hand-written header; and a `layout_contract_types!` row (`horus_cpp/src/layout_contract.rs:160`). They do not stay in sync — the module records the seven registry counts as 91 / 75 / 75 / 68 / 62 / 61 / 60 (`msgspec/mod.rs:10-11`). |
| **A message type for one project** | that project's `msgs/` directory | Nothing in this repository. `horus_manager/src/msgspec/` parses it and emits the Rust, Python, C++ and FFI artifacts from one definition (`msgspec/mod.rs:16`, `:35-42`, `:46-47`). |
| **A message type in user Rust code** | the user's own crate | The `message!` macro at `horus_core/src/communication/macros.rs:120-121`. |
| **A platform backend** | `horus_sys/src/<module>/<os>.rs` | Add the `cfg` arm and the re-export in that module's `mod.rs`. `shm/mod.rs:12-28` is the pattern. Check first whether the module you are extending is split into per-platform files at all — six of the ten are not. |
| **A hardware driver** | `horus_core/src/drivers/` | Three routes, and the obvious one is not portable. `register_driver!` hard-codes `#[link_section = ".init_array"]` with no `cfg` (`drivers/registry.rs:123-129`), so it is ELF-only; `drivers::registry::register` (`registry.rs:53`) is the portable call. For a driver that is a separate binary, use `use = "exec:<path>"` under `[hardware]` — dispatch is at `drivers/mod.rs:217-229` — and note that eleven keys (`use`, `sim`, `args`, `terra`, `package`, `node`, `crate`, `source`, `pip`, `exec`, `simulated`) are reserved rather than passed through as `NodeParams` (`drivers/mod.rs:111-124`, filtered at `:208-212`). |
| **A Python binding** | `horus_py/src/<name>.rs` | Register the class in `#[pymodule] fn _horus` (`horus_py/src/lib.rs:60-90`) and add the Python-side surface in `horus_py/horus/__init__.py` and `_horus.pyi`. |
| **A C++ binding** | `horus_cpp/src/<name>_ffi.rs` or `c_api.rs`, plus a header in `horus_cpp/include/horus/` | Not `horus_cpp_macros` — nothing depends on it (`horus_cpp_macros/src/lib.rs:3-9`). If the binding carries a message struct, add it to the layout contract and regenerate the header. |
| **A benchmark** | `benchmarks/benches/` or `benchmarks/src/bin/` | — |
| **An example project** | `examples/<name>/` | Twelve exist today. Each has its own `horus.toml` and a generated `.horus/`; none has a top-level `Cargo.toml`, and none is a workspace member. |
| **A workspace member** | a new top-level directory | Add it to `Cargo.toml:3-15`, and set `rust-version.workspace = true` and `[lints] workspace = true` in its manifest or it inherits neither. |

Two things that look like seams and are not:

- **`PoolBackend`** is documented as an open trait users can implement "without
  modifying horus_core" (`memory/backend.rs:18-21`), and a public constructor
  that takes one does exist: `TensorPool::with_backend`
  (`memory/tensor_pool.rs:674`). Nothing calls it — the only reference in the
  whole tree is its own doc example at `:672`. Every topic pool goes through
  `pool_registry::get_or_create_pool` (`communication/topic/pool_registry.rs:56-72`),
  which calls `TensorPool::open(pid)` or `TensorPool::new(pid, auto_pool_config())`
  and takes no backend argument. You can write a backend and construct a pool
  with it by hand; you cannot make a topic use one.
- **`Transport`**, as above. Public trait, concrete field.

## The horus_sys layering rule

`horus_sys/src/lib.rs:4-5` states the rule: all `#[cfg(target_os)]` code lives in
`horus_sys`, and `horus_core` and `horus_manager` never use platform APIs
directly.

**The rule is the right one and it is not what the tree does.** Measure it:

```sh
grep -rn '#\[cfg(target_os' <crate>/src | wc -l
```

run over all eleven members:

| Crate | `#[cfg(target_os` sites | of which test-only |
|---|---:|---:|
| `horus_sys` | 103 | 0 |
| `horus_core` | 26 | 4 |
| `horus_manager` | 17 | 9 |
| `benchmarks` | 17 | 0 |
| `horus_net` | 4 | 0 |
| `horus`, `horus_macros`, `horus_cpp_macros`, `horus_cpp`, `horus_types`, `horus_py` | 0 | — |

**64 sites sit outside `horus_sys`; 43 of them are in the two crates the rule
names.** Where they are, exactly:

- `horus_core` (26): `communication/mod.rs` 4, `communication/topic/header.rs` 3,
  `core/node.rs` 5, `core/presence.rs` 2, `core/rt_config.rs` 2,
  `scheduling/compute_executor.rs` 1, `scheduling/event_executor.rs` 2,
  `scheduling/rt_executor.rs` 2, `scheduling/rt.rs` 1,
  `testing/shm_fault.rs` 4. The last file compiles only under `cfg(test)` or
  `test-utils` (`lib.rs:50-52`), so the shipping count is 22.
- `horus_manager` (17): `commands/check.rs` 1, `commands/proxy.rs` 2,
  `discovery/tests.rs` 9, `plugins/executor.rs` 4, `plugins/mod.rs` 1.
  `discovery/tests.rs` is `#[cfg(test)]` (`discovery/mod.rs:288-289`), so the
  shipping count is 8.
- `benchmarks` (17): `platform.rs` 14, `lib.rs` 2, `bin/topic_probe.rs` 1. This
  is a measurement harness reading `/proc` and CPU topology, which is a weaker
  breach than the runtime ones — but it is a breach, and the rule's own text
  does not exempt it.
- `horus_net` (4): all in `event_loop/mod.rs`.

That count understates the branching, because `#[cfg(target_os` is not the only
spelling — `#[cfg(not(target_os = "linux"))]` does not match the grep at all, and
`rt_executor.rs:531` is exactly such a site. Counting `#[cfg(unix)]`,
`#[cfg(windows)]` and `#[cfg(not(unix))]` as well: `horus_core` 15/1/6,
`horus_manager` 78/15/4, `horus_net` 8/0/0, `benchmarks` 3/0/3, against
`horus_sys` 74/24/17.

And the sharpest signal is the dependency graph, not the attributes. `libc` is a
plain unconditional entry in `[dependencies]` for `horus_core`
(`horus_core/Cargo.toml:21`), `horus_manager` (`horus_manager/Cargo.toml:84`) and
`benchmarks` (`benchmarks/Cargo.toml:61`), while `horus_sys` gates it under
`[target.'cfg(unix)'.dependencies]` (`horus_sys/Cargo.toml:19-20`) and `horus_net`
under per-OS tables (`horus_net/Cargo.toml:15-19`). There are **38** `libc::`
call sites in `horus_core/src` and **33** in `horus_manager/src` — the largest
concentrations being `plugins/sandbox.rs` (21), `core/node.rs` (11),
`scheduling/rt_executor.rs` (7), `core/rt_config.rs` (7) and
`commands/proxy.rs` (8). For scale: `horus_sys` has 233 and `horus_net` 81, both
behind gated dependencies.

**What the rule actually is, stated honestly.** Two rules, not one:

1. *Platform API implementations belong in `horus_sys`.* This holds well.
   `horus_core/src/memory/platform.rs` is the model — a pure re-export shim over
   `horus_sys::shm` with no platform code of its own (`:1-19`; the remaining
   `:21-93` is its test module).
2. *A `cfg` branch in `horus_core` is acceptable when it selects between two
   shapes `horus_sys` already provides, and is not acceptable when it calls the
   platform itself.* `communication/mod.rs:77-158` is the acceptable kind: the
   Windows arm exists because a named section and a file-backed mmap are
   different Rust types (`:77-84`), and `open_named_section_mut` calls
   `horus_sys::shm::ShmRegion::open_existing` (`:149`) rather than the Win32 API.
   `scheduling/rt_executor.rs:495-570` is the other kind: `libc::clock_gettime`
   (`:523`) and `libc::clock_nanosleep` (`:551`) called directly, with
   `cfg(not(target_os = "linux"))` fallbacks beside them (`:531`, `:571`).

**Treat the number as a ratchet.** If your change adds a `#[cfg(target_os` site
outside `horus_sys`, the counts above must go up in this file in the same commit,
and you should be able to say in the PR why the code could not go behind a
`horus_sys` function. The strict-grep column is the thing a checker can enforce;
the other spellings and the `libc` counts are context, not the ratchet, and the
argument is the thing review should ask for.

## The public API boundary

`horus_core/README.md:3` says "**Internal implementation crate.** Use the `horus`
crate instead". `horus_core/src/lib.rs:15-16` repeats it, and `lib.rs:18-19`
states the policy: the modules are "accessible cross-crate but hidden from user
docs", and users should go through `horus::prelude` rather than importing from
`horus_core`.

What is actually public:

- **`horus_core` is re-exported wholesale by the umbrella crate.**
  `horus/src/lib.rs:353-354` is `#[doc(hidden)] pub use horus_core;`, followed by
  fourteen more top-level `pub use horus_core::…` lines through `:415`.
  `horus::horus_core` is a real, reachable path. `doc(hidden)` hides it from
  rustdoc; it does not make it private.
- **Every generated project depends on `horus_core` directly.**
  `horus_manager/src/cargo_gen.rs:1066` loops over
  `["horus", "horus_core", "horus_library", "horus_macros"]` and emits a path
  dependency for each one that exists on disk (`:1068`, `:1083-1090`). Confirmed
  in the generated files committed in this tree: nine `.horus/Cargo.toml` files
  under `examples/`, of which `examples/differential_drive/.horus/Cargo.toml:16`
  reads `horus_core = { path = "…/horus_core" }` under `[dependencies]`. A user
  who writes `use horus_core::…` in their node compiles.
- **The `[patch]` tables reach it too.** `Cargo.toml:108-115` and the mirrored
  `GIT_PATCH_TARGETS` in `cargo_gen.rs:1100-1109` are reproduced verbatim into
  every generated manifest (`examples/differential_drive/.horus/Cargo.toml:21-27`),
  including the horus-robotics/horus-tf asymmetry. A test,
  `patch_sections_match_root_workspace` (`cargo_gen.rs:2389`), keeps the constant
  and the root manifest in step.

So the honest statement is: **`horus_core` is public in the compiler's sense and
private in the documentation's sense.** `#[doc(hidden)]` appears 190 times across
25 files in `horus_core/src` (200 workspace-wide). It is a labelling convention,
not a boundary. Breaking a `pub` item in `horus_core` can break a user's project
even though nothing in the docs ever named it.

Consequences for a change:

- Adding a type users should reach means adding it to the prelude at
  `horus/src/lib.rs:581`. Being `pub` in `horus_core` is not enough to be
  discoverable, and being `doc(hidden)` in `horus_core` is not enough to be safe
  to break.
- The one deliberate exception is `horus_core::terminal`, which is `pub` and not
  hidden, with the reason written down at `lib.rs:38-44`.
- **Nothing is published to crates.io, and nothing can be.** `README.md:70-73`
  states it plainly: `cargo add horus` and `cargo install horus` fetch an
  unrelated crate that happens to own the name, and HORUS's own crates carry git
  and path dependencies that `cargo package` rejects, so none of them are
  published. No member manifest sets `publish` either way (only the three
  non-member fuzz and loom crates do). Semver is therefore enforced by review
  and by nothing else, and the member versions have already diverged: 0.4.0 for
  seven crates, 0.2.0 for `horus_sys`, 0.1.0 for `horus_cpp`,
  `horus_cpp_macros` and `horus_net`.

## The execution model

How a node reaches a thread. This determines what your change is allowed to
assume about ordering, and getting it wrong produces bugs whose only symptom is
slightly-wrong numbers.

**Step 1 — the node declares a class.** `ExecutionClass` has five variants
(`scheduling/types.rs:279-291`), default `BestEffort` (`:289-290`). The builder
sets it: `.compute()` (`node_builder.rs:220`), `.on(topic)` → `Event` (`:241`),
`.async_io()` (`:262`). Two things then promote a node to `Rt` in
`NodeBuilder::finalize` (`:502`), **but only if the class is still `BestEffort`**:
calling `.rate()` (`:523-527`) or setting an explicit budget or deadline
(`:544-550`). So `.rate(1000_u64.hz())` and nothing else gives you a real-time
node, while `.compute().rate(…)` stays `Compute` and the rate is only a limiter
(`:498-501`).

**Step 2 — the scheduler partitions.** `Scheduler::run` groups by class via
`group_nodes_by_class` (`scheduler/mod.rs:2912`, implemented at `types.rs:349-373`)
and hands each group to its executor (`scheduler/mod.rs:2869-2874`, `:3074-3169`).
Unless `.deterministic(true)` is set (`scheduler/mod.rs:666`), in which case
there is no partition at all and everything stays on the main thread
(`:2865-2867`).

| Class | Where it ticks |
|---|---|
| `Rt` | **One dedicated OS thread per node.** `RtExecutor::start_pool` (`rt_executor.rs:1066`) spawns per chain, and every chain is a singleton — `groups.rt_nodes.into_iter().map(\|n\| vec![n])` at `scheduler/mod.rs:3118-3119`. SCHED_FIFO where available. |
| `Compute` | A persistent worker pool with a coordinator thread; workers are spawned once at startup and parked, not per tick (`compute_executor.rs:1-22`, especially `:12`). |
| `Event(topic)` | One watcher thread per node, parked on the topic's `EventNotifier` and woken by `FUTEX_WAKE` from the publisher's `Topic::send()` (`event_executor.rs:1-29`). |
| `AsyncIo` | A dedicated thread running a tokio runtime; each tick goes through `spawn_blocking` (`async_executor.rs:1-25`). |
| `BestEffort` | Stays in `self.nodes` and is run by the main loop — but **in parallel**, not sequentially. With a dependency graph and more than one node, `execute_ready_dispatch` runs `total_to_tick.min(available_parallelism())` workers under `crossbeam::scope` (`scheduler/mod.rs:5651-5654`, `:5825-5830`, `:5836`), dispatching each node the moment its dependencies finish (`:5690-5702`). Sequential order is the fallback when there is no graph (`:5661-5682`) and the rule in deterministic mode (`:5641-5650`); a single node skips dispatch entirely (`:5655-5659`). The strategy is summarised at `:5606-5609`. |

The partition comment at `scheduler/mod.rs:2874` says BestEffort nodes stay in
`self.nodes` "for main-thread sequential execution". That describes the fallback,
not the default. Trust `:5651-5654`.

**The fact that changes how you write an RT node.** The RT executor gives **no
ordering guarantee between RT nodes**. It is stated in the source at
`scheduler/mod.rs:3113-3117`: an RT consumer may read its RT producer's
previous-tick value, at a phase offset fixed when the threads happened to start.
The scheduler detects RT→RT topic edges before the partition consumes the node
list and prints a warning naming both nodes (`scheduler/mod.rs:2894-2903`),
because the failure is invisible otherwise — the edge exists, the data flows,
every value looks plausible, and it is one cycle stale. On a legged robot that is
the estimator→controller edge and a real torque error (`:2889-2891`).

If you need two RT nodes ordered: put them in one node, or drive the chain
yourself with `tick_once()` (`scheduler/mod.rs:2904-2909`).

**`rt_executor.rs:2-22` contradicts this.** Its module diagram shows a single
"RT Thread (isolated)" ticking "RT node 0 / 1 / 2" in one loop, in priority
order. That was true of an earlier design and is not true of the code below it;
`scheduler/mod.rs:3104-3111` records that the grouping branch was dead
(`DependencyGraph::independent_chains` read no edges and was queried against the
post-partition graph, whose indices describe the BestEffort nodes) and has been
removed. Trust `scheduler/mod.rs:3097-3119`.

**Cores.** Each RT chain gets one core, resolved before any thread is spawned
(`rt_executor.rs:1084-1088`): an explicit `.core(n)` on any node in the chain
claims it for the whole chain, otherwise the scheduler's RT CPU list is assigned
round-robin — `rt_cpus[idx % rt_cpus.len()]` — and wraps silently when there are
more chains than CPUs (`rt_executor.rs:919-944`). With no list and no pin, the
chain runs unpinned and claims nothing (`:936-940`). A collision is a **clean
startup failure** when both chains named the core explicitly, or when
`HORUS_RT_WAIT=spin` is set — because two SCHED_FIFO chains on one core cannot
both meet their deadlines, and under spin the second chain does not run at all
and its watchdog latches an emergency stop (`rt_executor.rs:1004-1010`,
`:1013-1020`, error raised at `:1024-1029`). Override with
`HORUS_RT_ALLOW_CORE_SHARING=1` (`rt_executor.rs:899`, `:1030-1036`).

**Panic isolation.** Every tick, on every executor, runs inside `catch_unwind` —
`NodeRunner::run_tick` at `scheduling/primitives.rs:227-238`, called from the RT
(`rt_executor.rs:1240`), compute (`compute_executor.rs:208`), event
(`event_executor.rs:412`), async (`async_executor.rs:215`) and main-loop
(`scheduler/mod.rs:4872`, `:5882`) paths. That inner `catch_unwind` is the only
thing isolating a panicking `tick()`; the compute executor adds an *outer* one
around the surrounding machinery, and says why (`compute_executor.rs:199-209`).
It follows that `panic = "abort"` would turn per-node fault isolation off
entirely — a behavioural change, not an optimisation. No profile in this
workspace sets it today (`grep -rn 'panic *= *"abort"' --include=Cargo.toml`
returns nothing outside `target/`).

**Thread-local context.** `horus::now()`, `dt()`, `elapsed()`, `rng()` and
`budget_remaining()`, and the `hlog!` node context, are thread-locals installed
per tick (`scheduling/primitives.rs:241-250`). They must be installed on the same
thread that will run the tick, so a new executor must install them or those calls
return the inert fallbacks — `dt()`→0, `budget_remaining()`→`MAX`, `rng()`
unseeded (`:248-250`).

## What this file does not cover, and where that leaves you

Named plainly, because a map that implies full coverage is worse than one that
admits its edges:

- **The build and CI gates.** Eighteen workflows live in `.github/workflows/`.
  `CONTRIBUTING.md` disagrees with them on the two checks a newcomer meets first:
  it calls `cargo fmt` "optional, as it may fail in some cases" and clippy
  warnings "acceptable" (`CONTRIBUTING.md:121-122`, repeated at `:182` and
  `:232-233`), while `ci.yml:93` runs `cargo fmt --all -- --check` as its own
  job and `ci.yml:135-136` run clippy under `-D warnings` on `main` and on PRs
  targeting `main` (`RUSTFLAGS` set at `ci.yml:28`). Read the YAML, not the prose.
- **The test tree.** 114,496 lines of it. Which suite a new test belongs in,
  which loom models run in CI (`ci.yml:349` onward) and which are wired to
  nothing, and what CI deliberately skips.
- **The shared-memory wire format.** Changing `TopicHeader` or slot geometry has
  a procedure and out-of-crate readers that the compile-time asserts do not
  cover. Do not change `horus_core/src/communication/topic/header.rs` or
  `shm_layout.rs` from this document alone.
- **The `horus_net` protocol.** Twelve source modules and one test cite a
  `blueprint section N` that is not in this repository, and
  `safety_tests.rs:150` pins config defaults to it by name.
- **Release and versioning.** What `scripts/release.sh` rewrites, and which of
  the divergent version numbers move.
- **`horus_core` internals.** The best writing in the tree is in module headers
  that render nowhere. CI does build rustdoc — `cargo doc --workspace --no-deps
  --exclude horus_manager` (`ci.yml:345`) — but without
  `--document-private-items`, and no workflow deploys the output: none of the
  eighteen mentions `gh-pages`, `actions/deploy-pages` or `docs.rs`. So the
  topic safety model, the dispatch invariants and the RT allocator's no-alloc
  contract are readable only by opening the files or by running
  `cargo doc --document-private-items` yourself.

Two more honest gaps in *this* file. The module tables above say what each
`horus_core` module owns, not how the modules depend on each other, so they will
not tell you whether your change creates a cycle. And per-crate test-target
counts are deliberately absent: the numbers move between measurements and belong
to the test-tree document, not this one.
