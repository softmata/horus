# HORUS contributor documentation

Documentation for people changing HORUS. For documentation on *using* it, see
the documentation site.

Start with [`../CONTRIBUTING.md`](../CONTRIBUTING.md), which covers the
process — the branch model, what blocks a merge, and how to reproduce a CI
failure locally. The files here answer the questions it does not.

| Document | Answers |
|---|---|
| [`ARCHITECTURE.md`](ARCHITECTURE.md) | Where does my change go? The crate and module map, the public-API boundary, and which thread a node's `tick()` actually runs on. |
| [`TESTING.md`](TESTING.md) | Which suite does my test belong in? What CI invokes, what the `--test '*'` sweep reaches, and what it does not. |
| [`SHM_WIRE_FORMAT.md`](SHM_WIRE_FORMAT.md) | The procedure for changing the shared-memory layout, and the readers outside `horus_core` that have to move with it. |
| [`RELEASING.md`](RELEASING.md) | Cutting a release: what the script rewrites, what the tag triggers, and the gates it then faces. |

Two more live next to the code they describe:

- [`../horus_net/docs/BLUEPRINT.md`](../horus_net/docs/BLUEPRINT.md) — the
  network protocol specification that thirteen `horus_net` modules cite by
  section number.
- [`../horus_core/README.md`](../horus_core/README.md) — an index into
  `horus_core`'s design documentation. Most of that crate's modules are
  `#[doc(hidden)]`, so `cargo doc` renders almost none of it.

## Why these are in the repository

They are statements about the code, so they belong in the same tree as the
code and in the same pull request as the change they describe. A fork gets
them; a wiki page would not, and neither would be reachable from a CI check.

That is not a hypothetical concern here. `CONTRIBUTING.md` spent long enough
telling contributors that `cargo fmt` was optional and that clippy warnings
were acceptable — while CI ran `cargo fmt --all -- --check` unconditionally and
`clippy -D warnings` against `main` — for the advice to be actively costing
people merges. Documentation that nothing checks drifts, and drifted
documentation about a merge gate is worse than none.
