# Cutting a release, and adding a workspace member

HORUS ships through four channels — GitHub release assets, PyPI, GHCR and the
source tree `install.sh` caches — and they are cut by three workflows that do
not know about each other. Nothing coordinates them. When one succeeds and
another fails, the result is not a failed release; it is a release that is half
true, and the halves disagree about what version the software is.

That is not hypothetical. As of 2026-09-03 the newest GitHub release is
`v0.4.0`, published 2026-08-23T01:07:50Z with five binaries and a `SHA256SUMS`
(`gh api repos/softmata/horus/releases/latest`), and
`https://pypi.org/pypi/horus-robotics/json` serves `0.1.9` — the only versions
PyPI holds at all are 0.1.6, 0.1.7 and 0.1.9. The `build-wheels.yml` run for
that same tag (run 32609224637) built every wheel, passed all twelve
`Test Installation` legs, and then failed on `Publish to PyPI`.
`build-wheels.yml:14` records that PyPI sat on 0.1.9 for five months before
that. The 0.3.0 tag was cut locally and never pushed — `git tag` in this
checkout lists `alpha` and `v0.1.5` through `v0.4.0` with no `v0.3.0` — and
because `install.sh:307-308` resolves `releases/latest`, every user installing
during that window got 0.2.2 with nothing to tell them so (`CHANGELOG.md:41-46`).

Read this before you cut a tag. The section you most need is
[When one leg fails after another has published](#when-one-leg-fails-after-another-has-published),
because that is the state a HORUS release most often ends in.

---

## What this document does not cover, up front

- **The current gates have never run against a real tag.** `release.yml`,
  `build-wheels.yml` and `container-images.yml` were all last modified in commit
  `a9540b5` on 2026-09-02, after the only successful release run (v0.4.0,
  2026-08-23). Every gate described below is read from the file, not from a run
  that used it. Two concrete signs of the gap: the published v0.4.0 carries no
  `horus-linux-armv7.tar.gz`, which the current asset check at
  `release.yml:299-327` would refuse to publish without; and the v0.4.0
  `build-wheels.yml` run has no `Verify Tag Matches Package Version` job and no
  `Verify PyPI Publication` job in it at all, because neither existed yet
  (`gh run view 32609224637 --json jobs`).
- **`scripts/release.sh` is stale and nothing enforces otherwise.** Its last
  commit is `753d682`, 2026-04-23. `grep -rn 'release\.sh' .` over the tree
  (excluding only `target/`, `.git/` and `node_modules/`) returns two hits, both
  inside `scripts/release.sh` itself (`:17`, `:18`). No workflow invokes it, no
  test reads it, no markdown file mentions it. Most of its rewrite list no
  longer exists on disk — see
  [What release.sh actually rewrites](#what-releasesh-actually-rewrites).
- **`container-images.yml`'s publish path has never executed.** Every `push`
  run of that workflow is on `main`; none is on a tag
  (`gh run list --workflow=container-images.yml --json headBranch,event`).
  Whether `ghcr.io/softmata/horus` holds anything is **unverified**: the
  anonymous GHCR token endpoint returns 403 (`DENIED`), and
  `gh api '/orgs/softmata/packages?package_type=container'` returns 403 for want
  of the `read:packages` scope. What is verifiable is only that the workflow's
  publish steps have never had a tag ref to run against.
- Nothing here covers `horus-sim2d`, `horus-sim3d`, `horus_library`, the docs
  site, `horus-robotics` or `horus-tf`. They are separate repositories with
  separate versioning (`scripts/release.sh:15`, `:115-116`, `:133`, `:228`,
  `:241-242`; `Cargo.toml:40`, `:108-115`).

---

## Where the version numbers live

Eleven crates are workspace members (`Cargo.toml:3-15`). Their versions do not
all move together, and that is deliberate:

| Member | Version on disk | Moves with the release? |
|---|---|---|
| `horus` | 0.4.0 | yes |
| `horus_core` | 0.4.0 | yes |
| `horus_macros` | 0.4.0 | yes |
| `horus_manager` | 0.4.0 | yes |
| `horus_types` | 0.4.0 | yes, by hand — see below |
| `horus_py` | 0.4.0 | yes |
| `benchmarks` (package `horus_benchmarks`) | 0.4.0 | yes |
| `horus_sys` | 0.2.0 | only when its shared-memory header changes |
| `horus_net` | 0.1.0 | no |
| `horus_cpp` | 0.1.0 | no |
| `horus_cpp_macros` | 0.1.0 | no |

Each version is at `<member>/Cargo.toml:3`. The member directory `benchmarks`
holds a package named `horus_benchmarks` (`benchmarks/Cargo.toml:2`); the
directory name is what the workspace list and the Dockerfile use.

`CHANGELOG.md:48-51` states the rule: the first seven move together, `horus_sys`
moved 0.1.0 → 0.2.0 at the 0.4.0 release because `TopicHeader` changed, and the
last three stay at 0.1.0.

Four different pieces of the release machinery each read a *different* manifest
as the authoritative version. If those manifests disagree, each one breaks
somewhere else:

| Reader | Manifest it reads | What breaks when it disagrees |
|---|---|---|
| `scripts/release.sh:58` | `horus/Cargo.toml` | every `sed` in the script matches nothing, silently |
| `install.sh:462` | `horus_core/Cargo.toml` | names the cache dir `~/.horus/cache/horus@<v>` (`install.sh:260`, `:475`); a wrong value points `horus build` at a tree that is not there |
| `install.sh:764-774` | `horus_manager/Cargo.toml` | the install aborts with `Version skew` — a hard `exit 1` for every user |
| `build-wheels.yml:69` | `horus_py/pyproject.toml` | this is what maturin stamps on the wheel (`build-wheels.yml:49-51`); the tag is compared against it |

`horus_py/Cargo.toml` is cross-checked against `horus_py/pyproject.toml` at
`build-wheels.yml:76-80`, tolerating `version.workspace = true`.

**The consequence worth memorising:** `horus_core` and `horus_manager` must
carry the same version, or `install.sh` refuses to install for everybody
(`install.sh:767-774`).

`horus --version` is not a hand-maintained string. `horus_manager/src/main.rs:14`
declares `#[command(version = env!("CARGO_PKG_VERSION"))]`, and fourteen
`env!("CARGO_PKG_VERSION")` sites across `horus_manager/src` read it directly.
Bumping `horus_manager/Cargo.toml` is the whole of it.

One more manifest is tied to that number by a test that gates merges:
`manifest_contract.rs:293-313` asserts the root `horus.toml`'s
`[package] version` equals `horus_manager`'s `CARGO_PKG_VERSION`, with the
reason in its doc comment — "HORUS's own manifest declared 0.2.0 against a 0.3.0
binary. Nothing tied them together, so it drifted the moment a release was cut."
See [Version checks that run at merge](#version-checks-that-run-at-merge-not-at-tag).

### Version strings that are carried by hand

`README.md` holds seven occurrences of `0.4.0` (lines 15, 49, 63, 68, 517, 595,
605). No test compares any of them to a manifest: the only `CARGO_PKG_VERSION`
comparisons anywhere under a `tests/` directory are `changelog_contract.rs:89`
(against the changelog) and `manifest_contract.rs:305-312` (against
`horus.toml`); the third hit, `install_contract.rs:1451`, is a comment. One of
the seven README occurrences is a trap:

> `README.md:595` — "Releases cut before that floor existed, `v0.4.0` and
> earlier, were built natively on the CI runner and need glibc 2.39"

That sentence is *about* 0.4.0 as a historical fact. `scripts/release.sh:249`
runs an unanchored `sed -i "s/$CURRENT_VERSION/$NEW_VERSION/g"` over the whole
of `README.md`, so running the script would rewrite it into a false statement
about the release you are cutting. The same blunt global substitution is applied
to every file in the TOML list (`release.sh:233`). Note also that the dots in
`$CURRENT_VERSION` reach `sed` unescaped, so the pattern is a regex in which
each `.` matches any character.

---

## What `release.sh` actually rewrites

The script is a ten-step `sed` driver. Its header claims "Cargo.toml package
versions (10 crates)" (`release.sh:6`); the comment above the array claims 11
files (`release.sh:98`); the array holds nine paths (`release.sh:100-117`). The
array is what runs.

| Step | Lines | Paths named | Exist on disk | Effect today |
|---|---|---|---|---|
| 1 — crate manifests | `:100-126` | 9 | 6 | bumps `horus`, `horus_core`, `horus_macros`, `horus_manager`, `horus_py`, `benchmarks`. `horus_router`, `horus_library`, `horus_ai` print `- (not found)` |
| 2 — pyproject | `:131-141` | 1 | 1 | bumps `horus_py/pyproject.toml` |
| 3 — Python `__version__` | `:146-157` | 2 | 1 | **rewrites nothing.** `horus_py/horus/__init__.py:299` derives the version from Rust; `:301` is a `"0.2.0"` fallback the pattern cannot match. `horus_py/benchmarks/__init__.py` does not exist |
| 4 — Rust sources | `:163-210` | 8 | 3 | **rewrites nothing.** Five paths are gone (`commands/run.rs`, `registry.rs`, `monitor.rs`, `monitor_tui.rs`, `config/cargo_config.rs`); the three that remain (`main.rs`, `commands/new.rs`, `workspace.rs`) hold no matching literal |
| 5 — TOML configs | `:215-236` | 12 | 2 | bumps `horus.toml:3` only — the one occurrence of `0.4.0` in that file, and load-bearing, see `manifest_contract.rs:293-313`. `horus_core/tests/horus.toml` declares `version = "0.2.0"` and holds no `0.4.0`; the other ten paths are gone |
| 6 — package.json | `:239-242` | 0 | — | prints a note; docs-site is a separate repository |
| 7 — documentation | `:248-259` | 4 | 3 | rewrites `README.md` (see the trap above). `horus_py/README.md` and `horus_manager/README.md` hold no `0.4.0`; `horus_library/README.md` is gone |
| 8 — templates | `:269-278` | 2 | 2 | **rewrites nothing.** `bug_report.yml:56` reads `placeholder: "0.2.0"`, already two releases behind, so the pattern misses. `.github/PYPI_SETUP.md` is full of version strings, but all of them are 0.1.x or 0.2.0 (`:67-68`, `:72`, `:75`, `:99`, `:149`, `:155-157`), and the pattern looks for a backticked `` `0.4.0` `` |
| 9 — Python tests | `:284-289` | glob | — | finds files by content rather than by path, so it does not rot — but none of the nineteen `*/tests/*.py` files currently contains `version = "0.4.0"`, so it matches nothing today |
| 10 — git | `:294-321` | — | — | `git add -A`, commit, `git tag v<version>` — **it does not push** |

Two properties of this script matter more than the table.

**A green tick does not mean a rewrite happened.** Steps 2, 3, 4, 5, 7 and 8
print `+ <file>` whenever the file exists, regardless of whether `sed` matched
anything (`release.sh:136-141`, `:152-157`, `:163-210`, `:231-236`, `:248-259`,
`:269-278`). Only step 1 distinguishes the two, and only by reporting a missing
file (`:119-126`); step 9 is the one place a tick means a match, because it
greps before it substitutes. The run will look entirely successful while
changing nine files out of the thirty-eight it names: six crate manifests,
`horus_py/pyproject.toml`, `horus.toml`, `README.md`.

**It cannot produce the version spread that 0.4.0 actually shipped.**
`CHANGELOG.md:48-51` records `horus_types` moving to 0.4.0 and `horus_sys`
moving 0.1.0 → 0.2.0. Neither crate is in `CARGO_FILES`, and neither are
`horus_cpp`, `horus_cpp_macros` or `horus_net`. Whatever moved those numbers, it
was not this script alone.

**It does not touch `CHANGELOG.md`.** `CHANGELOG.md:34-35` says every entry
under `Unreleased` moves under the new version heading at cut time and
`Unreleased` is left empty rather than deleted. That is a manual step. Do it
before you tag, not after — the tag is what people read the changelog against.

### If you run it anyway

```
./scripts/release.sh 0.5.0
```

It validates the argument as `X.Y.Z` or `X.Y.Z-suffix` (`release.sh:44-49`),
refuses to run outside the repository root (`:52-55`), warns on a dirty tree and
asks (`:69-79`), refuses if `v0.5.0` already exists (`:82-85`), and asks once
more before writing (`:88-93`). The undo it prints is local-only and works only
before you push (`release.sh:346`):

```
git reset --hard HEAD~1 && git tag -d v0.5.0
```

Review `git show HEAD` against the table above and hand-edit whatever the script
missed. In practice that is the point of running it: it gets six crate manifests
right and gives you a commit to correct.

---

## The tag

The shape is `v*.*.*` — `release.yml:4-6`, `build-wheels.yml:4-6`,
`container-images.yml:43-44`. All three use the same glob, so a tag either
starts all three or none of them. `alpha` and any non-`v` tag start nothing.

`release.sh:334` tells you to push with:

```
git push origin main --tags
```

`main` is protected: seven required contexts, `strict: true`,
`enforce_admins: false`, `required_linear_history: true`, no force pushes, no
branch deletions, required conversation resolution, and no
`required_pull_request_reviews` block at all
(`gh api repos/softmata/horus/branches/main/protection`). With admin enforcement
off, a repository administrator can push the release commit directly. Anyone
else has to land it through a pull request first and then push the tag alone.

Tags themselves are unprotected: `gh api repos/softmata/horus/tags/protection`
returns 404, and the only ruleset on the repository
(`gh api repos/softmata/horus/rulesets`) is "Code Quality Copilot review for
default branch", whose target is `branch`. Deleting and re-pushing a tag is
therefore possible, which the recovery advice below relies on.

**No merge gate runs on the tag.** None of the seven required-context workflows
declares a `tags:` filter — `ci.yml:6-10`, `multi-platform.yml:6-15`,
`integration-tests.yml:7-12`, `feature-matrix.yml:6-14`,
`cross-platform-parity.yml:14-19`, `docker-distro-tests.yml:8-17`,
`benchmarks.yml:3-11`. They trigger on `pull_request` and on `push` to branches;
five of them also carry a weekly cron and `workflow_dispatch`, and none of those
paths sees a tag either. A tag inherits whatever CI ran when its commit landed
on `main`, and nothing re-checks it. If you amend anything after that commit
went green, you are tagging untested code.

---

## The gates a tag faces

| Gate | Where | What it refuses | Runs on a tag |
|---|---|---|---|
| Tag matches the Python package version | `build-wheels.yml:56-93` | a tag whose commit did not bump `horus_py/pyproject.toml`, and a `horus_py/Cargo.toml` that disagrees with it | yes, first, and everything else `needs` it |
| Complete asset set | `release.yml:299-327` | a Release missing any of the six named assets, or holding a different count from the list | yes, before the Release is created |
| glibc floor, statically | `release.yml:196-211` | a Linux binary whose highest `GLIBC_` version-need exceeds `GLIBC_FLOOR` (`release.yml:26`, `2.28`), read back out of the ELF with `readelf` | yes, per Linux leg, before upload |
| glibc floor, dynamically | `release.yml:232-240` | a Linux tarball that cannot unpack and run `--version` inside `debian:11`, under qemu for arm64 and armv7 (`release.yml:213-217`) | yes, per Linux leg, before upload |
| Wheel imports its extension | `build-wheels.yml:306-317` | a wheel where `import horus._horus` fails or `horus.Topic` is `None` — plain `import horus` passes even in mock mode | yes, twelve legs, and `Publish to PyPI` needs it |
| Image carries the tagged binary | `container-images.yml:228-236` | a `v0.4.0` image whose `horus:cli --version` reports anything else | yes, before the GHCR login |
| PyPI actually serves it | `build-wheels.yml:247-276` | a green upload that pypi.org is not serving five minutes later | yes, after the publish |

The Linux floor is enforced twice on purpose. `release.yml:19-26` explains why:
the `.2.28` suffix on each `zig_target` tells zig which glibc stubs to link
against, and the `readelf` step reads the version-needs back out so a drifting
toolchain fails the release instead of shipping. The three Linux legs
cross-compile with `cargo-zigbuild`, pinned exactly (`cargo-zigbuild==0.23.3`,
`ziglang==0.16.0`, `release.yml:151-158`) because a zig upgrade would move the
floor of every asset without a line of the workflow changing.

### What is not gated on the tag

- **`distribution.yml` does not trigger on tags at all.** Its triggers are
  `pull_request` on `[main, dev]`, `push` on `[main]`, a daily 06:00 cron, and
  `workflow_dispatch` (`distribution.yml:28-40`). So `tag-coherence`
  (`:452-583`) — the job that installs the resolved tag and asserts the binary,
  `~/.horus/install_manifest.toml` and the cached source tree all name the same
  commit and the same `TOPIC_VERSION` (`:547-564`) — sees your release the *next
  morning*, or on the next pull request. It is a standing check on the published
  state, not a pre-publication gate.
- **`Distribution CI Success` is not a required context.** The seven required
  contexts are `CI Success` (`ci.yml:480`), `Multi-Platform Success`
  (`multi-platform.yml:403`), `Integration Tests Success`
  (`integration-tests.yml:1372`), `Feature Matrix Success`
  (`feature-matrix.yml:154`), `Parity CI Success`
  (`cross-platform-parity.yml:341`), `All Distros Pass`
  (`docker-distro-tests.yml:64`) and `Run Benchmarks` (`benchmarks.yml:70`).
  `distribution.yml:656-686` builds a gate job that is bound to nothing.
- **The wheel's own glibc floor is unverified.** `release.yml:20` claims the
  wheels ask maturin for the same floor with `manylinux: auto`, and they do
  (`build-wheels.yml:115`), but there is no `auditwheel` or `readelf` step
  anywhere in that workflow — `manylinux: auto` at `:115` is the sole match for
  `glibc|auditwheel|manylinux|GLIBC` in the whole file. The CLI's floor is
  proved; the wheel's is asserted.
- **No armv7 wheel.** `release.yml:84-90` builds `horus-linux-armv7`;
  `build-wheels.yml:99-101` builds Linux wheels for `[x86_64, aarch64]` only. A
  32-bit Raspberry Pi gets a CLI and no `pip install`.

### Version checks that run at merge, not at tag

Three Rust contract tests police release hygiene. They run — but on pull
requests and pushes to `main`, so they see your release only after it is out.

`integration-tests.yml:127` runs
`cargo test --workspace --exclude horus_py --release --test '*'` (with five
`--skip` arguments for timing-sensitive stress tests, `:129-133`), a glob over
every integration-test target in every member but `horus_py`. That job,
`Cross-Process IPC Tests` (`:38-39`), feeds `Integration Tests Success`
(`:1371-1374`), a required context. **No workflow names these suites** —
`grep -rn 'changelog_contract\|manifest_contract' .github/workflows/` finds
nothing, and the only contract suites named anywhere in a workflow are
`install_contract` and `completion_install_contract`
(`cross-platform-parity.yml:141-142`) — so it is easy to conclude they run
nowhere. They do. In run 33810476114, job `Cross-Process IPC Tests`, the log
carries `Running tests/changelog_contract.rs` followed by
`test result: ok. 9 passed`, and `Running tests/manifest_contract.rs` followed
by `test result: ok. 11 passed`.

| Check | Where | What it catches | Blind spot |
|---|---|---|---|
| `the_repository_manifest_states_the_current_version` | `manifest_contract.rs:293-313` | `horus.toml` left behind by a version bump | none — it parses the TOML and compares the two directly |
| `every_released_tag_appears_in_the_changelog` | `changelog_contract.rs:62-83` | a tag with no `[X.Y.Z]` section | **vacuous in CI.** `actions/checkout` fetches no tags (`integration-tests.yml:44`, no `with:`), so `git tag` returns empty, the test takes its skip branch (`:64-68`), prints `SKIP: no git tags visible (not a git checkout?)` and reports `ok`. Both lines are in the run log above, consecutively |
| `the_current_version_has_a_home` | `changelog_contract.rs:87-97` | nothing that matters | passes whenever `## Unreleased` exists, and `CHANGELOG.md:37` always has one. It does not force a section for the version being cut |

So the changelog entry for a release is enforced by nobody. Write it because the
next person deciding whether to upgrade has no other source, not because a check
will stop you.

Because these tests are selected by a glob rather than by name, narrowing that
glob, or adding an `--exclude`, drops them silently and no grep will reveal it.

---

## Order of operations

1. Move `CHANGELOG.md`'s `Unreleased` entries under a new `## [X.Y.Z] — <date>`
   heading and leave `Unreleased` empty (`CHANGELOG.md:34-35`). State which
   crate versions move, as `CHANGELOG.md:48-51` does.
2. Bump the versions. Run `./scripts/release.sh X.Y.Z` if you want the six crate
   manifests it still gets right, then correct the rest by hand against the
   table above. Four constraints bind: `horus_core` and `horus_manager` must
   match (`install.sh:767-774`); `horus_py/Cargo.toml` and
   `horus_py/pyproject.toml` must match (`build-wheels.yml:76-80`);
   `horus_py/pyproject.toml` must equal the tag without its `v`
   (`build-wheels.yml:89-92`); and `horus.toml`'s `[package] version` must equal
   `horus_manager`'s (`manifest_contract.rs:293-313`, which gates merges).
3. Land the commit on `main` the normal way and let the seven required contexts
   go green. This is the *only* full test run the release gets.
4. Tag that commit and push the tag.
5. Three workflows start concurrently, with no ordering between them:
   - `release.yml` — six-leg matrix, `fail-fast: false` (`:50`), then
     `create-release`, which verifies the asset set, computes `SHA256SUMS`,
     attests provenance and creates the Release (`:299-385`).
   - `build-wheels.yml` — `version-guard`, then four build jobs over six legs
     (`linux` ×2 at `:95-101`, `macos` ×2 at `:124-130`, `windows` ×1 at
     `:152-158`, `sdist` at `:180`), then twelve `test-install` legs
     (`:278-285`), then `Publish to PyPI` (which `needs` all of them, `:206`),
     then `verify-pypi`.
   - `container-images.yml` — builds `horus:cli` (`:102`), `horus:dev` (`:119`)
     and the devcontainer stage (`:160`), smoke-tests all three, then pushes
     four GHCR refs and pulls one back (`:263-288`).
6. `create-release` publishing a Release does **not** re-trigger
   `build-wheels.yml`'s `release: [published]` listener. That listener exists
   for the six tags GitHub never delivered a `push` event for
   (`build-wheels.yml:7-20`), and events raised by `GITHUB_TOKEN` never start
   new runs. The Release that `create-release` cuts is authored by
   `github-actions[bot]` — which is what `gh api repos/softmata/horus/releases/latest`
   reports for v0.4.0.
7. Confirm all three. There is no job anywhere that checks the three channels
   agree: `pypi` appears in only three workflow files, and in `ci.yml:212` and
   `release.yml:147` it is a comment; `ghcr` appears only in
   `container-images.yml`.

A dry run is available: `release.yml`'s `workflow_dispatch` with a blank `tag`
input builds the whole matrix and skips `create-release`
(`release.yml:10-15`, `:276-279`).

---

## When one leg fails after another has published

The three channels differ in one property that decides everything: whether you
can publish the same version twice.

| Channel | Re-publishable under the same version | Why |
|---|---|---|
| GitHub Release | yes | the step is named "Create / update Release" and takes `tag_name` (`release.yml:377-385`) |
| GHCR | yes | `docker push` overwrites a mutable tag (`container-images.yml:263-271`) |
| PyPI | **no** | `build-wheels.yml:239-240` deliberately omits `--skip-existing`: "a version already on PyPI means the tag did not bump `horus_py/pyproject.toml`, and that must be a red run, not a quiet pass" |

The PyPI row is read from this repository's own configuration, not from PyPI's
published rules; treat "cannot" as "this workflow will go red, and you should
assume the version number is spent".

So:

**A `release.yml` build leg failed.** Nothing published. `create-release` is
gated on `success()` and on the complete asset set (`release.yml:276-279`,
`:299-327`), so an incomplete matrix produces no Release and no assets. Fix the
cause, then re-run with `workflow_dispatch` and the `tag` input set — that path
runs the full matrix and then creates the Release for that tag
(`release.yml:276-279`, `:360-385`). No tag surgery needed.

**`release.yml` went green but `build-wheels.yml` failed.** This is the live
state of v0.4.0. Binaries exist, PyPI does not have them. `install.sh` resolves
`releases/latest` through the plain redirect (`install.sh:305-315`) and then
downloads from `releases/download/<tag>/` (`:393-394`), and `horus self update`
resolves the API's `releases/latest`
(`horus_manager/src/commands/upgrade.rs:130`), so the binary channel is correct
and only `pip install horus-robotics` is stale. What to do depends on where it
failed:

- *`version-guard` failed* — nothing was built or uploaded. The fix changes the
  commit, so the tag has to move. Delete and re-push the tag; that re-fires all
  three workflows, and both the Release and the GHCR tags update in place.
- *A build or `test-install` leg failed* — `Publish to PyPI` `needs` all of them
  (`build-wheels.yml:206`), so nothing was uploaded. Fix, and re-run the failed
  jobs.
- *`Publish to PyPI` failed* — check whether any file reached PyPI before it
  died. If none did, re-run the job. If any did, treat that version number as
  spent: bump `horus_py/pyproject.toml`, cut a new tag, and say so in the
  changelog.
- *`verify-pypi` failed* — the upload succeeded and pypi.org was not serving the
  version within twenty polls at fifteen-second intervals
  (`build-wheels.yml:262-276`). The poll is idempotent; re-run that job alone
  before concluding anything.

**`container-images.yml` failed.** Every build and smoke-test step precedes the
GHCR login at `container-images.yml:240-246` and the push at `:248-271`, so a
failure before that means nothing was pushed. GHCR tags are mutable, so
re-running the workflow on the tag is always safe.

**The rule.** GitHub Releases and GHCR are recoverable in place; PyPI is a
one-way door. If you are unsure whether a partial upload happened, treat the
version as spent. A re-cut minor version costs a changelog entry; a
half-published PyPI version costs users an import error they cannot diagnose.

---

## Adding a workspace member

Seven things can need to learn about a new crate. Cargo enforces two on its own,
required tests cover two more, and three are covered by nothing.

| What must learn about it | Where | Enforced by |
|---|---|---|
| The workspace member list | `Cargo.toml:3-15` | cargo itself |
| A `[workspace.dependencies]` entry, if siblings depend on it | `Cargo.toml:30-39` | cargo, at the first `path` dep |
| `rust-version.workspace = true` in the new crate | all eleven members declare it | `install_contract.rs:203-243` `every_crate_inherits_the_workspace_rust_version` — **runs**, named at `cross-platform-parity.yml:137-142` in the `Parity (macOS ARM64)` job (`:85-86`), which feeds the required `Parity CI Success` (`:340-342`), and again through the glob at `integration-tests.yml:127` |
| `[lints]` / `workspace = true` in the new crate | all eleven members declare it | nothing. No test under any `tests/` directory reads a member's `[lints]` table |
| A `COPY <crate>/Cargo.toml` line in the Dockerfile | `Dockerfile:79-89` | `install_contract.rs:545-571` `the_dockerfile_copies_every_workspace_member` — **runs**, both routes. `changelog_contract.rs:193-242` holds a near-duplicate, `the_dockerfile_covers_every_workspace_member`, that also runs |
| A `cargo check -p <crate> --no-default-features` line, if the crate has default features | `feature-matrix.yml:111-115` | nothing |
| `CARGO_FILES` in the release script, if the crate versions in lockstep | `scripts/release.sh:100-117` | nothing |

On the Dockerfile line: omitting it does not actually break the image today.
`COPY . .` at `Dockerfile:91` precedes the only `cargo build` at `:93`, so the
per-member copies at `:79-89` affect layer caching and nothing else. The test
comments (`install_contract.rs:542-544`, `changelog_contract.rs:190-192`) both
predict a build failure that would not occur. Add the line anyway — the test is
a required check, and it will fail your PR.

On `feature-matrix.yml`: `--workspace --no-default-features` does not produce a
minimal build, because cargo unifies features across the workspace and siblings
re-enable them. The comment at `:105-110` says so, and `:111-115` lists the five
crates checked individually: `horus_core`, `horus`, `horus_types`, `horus_net`,
`horus_sys`. If your crate has default features and you do not add a line,
nothing checks it.

### The MSRV disagreement, which you will hit

`Cargo.toml:28` declares `rust-version = "1.90"`, and `Dockerfile:65` pins
`FROM rust:1.90-slim-bookworm`. `install_contract.rs:517-539`
(`the_dockerfile_pins_the_workspace_msrv`) asserts those two agree, and it is a
required check.

`multi-platform.yml:342-350` defines a job named `MSRV (${{ matrix.rust }})`
whose only matrix entry is `"1.92.0"`, annotated `# MSRV`.
`install_contract.rs:200-202` also refers to "the workspace's 1.92".

1.90 is what binds. It is what `rust-version` makes cargo enforce for anyone
depending on the crate, and it is the toolchain the Docker build compiles with.
The number in `multi-platform.yml` is two minor versions above the floor, so the
job named MSRV cannot fail for the reason its name implies — a crate that needs
1.92 would pass it and then fail the Docker build. That Docker build is
`container-images.yml:102`, which is not one of the seven required contexts and
on pull requests only runs when a path filter matches
(`container-images.yml:24-32`). `grep -rn '1\.90' .github/workflows/` returns
nothing and there is no `rust-toolchain.toml`, so **the declared floor is
compiled by no required check**. Write for 1.90, and expect to find out late if
you did not.

---

## The checker that must ship with this file

This document is described here, not written. It makes claims about paths and
version literals that rot the moment someone reorganises `horus_manager/src` —
which is exactly how `scripts/release.sh` reached its present state. It needs a
contract test in the idiom of `horus_manager/tests/changelog_contract.rs`:

- every path `scripts/release.sh` rewrites is listed in this file, and still
  exists on disk (so a path that disappears fails here instead of becoming a
  silent no-op);
- every version literal this file quotes matches the corresponding manifest;
- every workspace member in `Cargo.toml:3-15` appears in the version table above.

Put it in `horus_manager/tests/` and it is picked up automatically by the
`--test '*'` glob at `integration-tests.yml:127`, under the required
`Integration Tests Success` context. That is how `changelog_contract.rs`,
`readme_contract.rs`, `manifest_contract.rs`, `install_url_contract.rs` and
`translation_contract.rs` already run, despite no workflow naming any of them.

Do not take that inheritance as sufficient. `every_released_tag_appears_in_the_changelog`
is proof of the failure mode: it is selected by the glob, it executes, it
reports `ok`, and it asserts nothing, because the CI checkout has no tags for it
to read. A test that runs is not a test that checks. If a claim in this file
needs git history or a tag to verify, say so in the test and make it fail
loudly when it cannot, rather than skipping into a green tick.

---

## Where the sources disagree, and which one wins

| Disagreement | Winner |
|---|---|
| `release.sh:6` says 10 crates, `:98` says 11 files, the array `:100-117` holds 9 | the array; nothing reads the comments |
| `release.sh`'s file lists vs. the tree — 3 of 9 manifests, 5 of 8 Rust sources, 10 of 12 TOMLs and 1 of 4 READMEs are gone | the tree; step 1 prints `- (not found)`, the other steps say nothing |
| `release.sh:163-166` rewrites a `#[command(version = "…")]` literal; `main.rs:14` uses `env!("CARGO_PKG_VERSION")` | `main.rs`; the sed is dead and the version still moves via `horus_manager/Cargo.toml` |
| `CHANGELOG.md:48-51` lists `horus_types` and `horus_sys` as moving at 0.4.0; neither is in `release.sh`'s `CARGO_FILES` | the changelog describes what shipped; the script could not have produced it |
| `distribution.yml:381-392` says the `glibc-floor` job "must not gate merges"; `:678` requires its result to be `success` | `continue-on-error: true` at `:393`. Verified empirically: in run 33830955509, `Published tarball on debian:11` concluded `failure` while `Distribution CI Success` and the whole run concluded `success` |
| Workspace MSRV `1.90` (`Cargo.toml:28`, `Dockerfile:65`) vs. the job named MSRV at `1.92.0` (`multi-platform.yml:350`) | 1.90 binds; 1.92 is what CI compiles. See above |
| `grep -rn changelog_contract .github/workflows/` finds nothing, suggesting the suite runs nowhere; `integration-tests.yml:127` selects it with `--test '*'` | the glob. Run 33810476114 logs `Running tests/changelog_contract.rs`. A name-based grep is not a coverage answer in this repository |
| `release.sh:334` says `git push origin main --tags`; `main` requires seven contexts with `strict: true` | `enforce_admins: false`, so an administrator can push the release commit directly. For anyone else the required contexts apply, so the commit has to land through a pull request first. (That an ordinary push is rejected for non-administrators is standard branch-protection behaviour, not something read out of a file here) |

### Line-number citations in the workflows have drifted

The comments in these files cite each other by line, and those citations are not
checked by anything. Of the five cross-file citations spot-checked in the
release path, four are now wrong and one is right. The pattern is the point, not
the individual numbers:

| Citation | State |
|---|---|
| `release.yml:20` cites `build-wheels.yml:42` for `manylinux: auto` | drifted; that setting is at `build-wheels.yml:115`, and `:42` is a comment about concurrency. The claim itself still holds |
| `distribution.yml:432` cites `release.yml:156-164` for a smoke test on the build host | drifted; `release.yml:156-164` is now the tail of the zig install and the cargo cache step. `release.yml` runs the Linux smoke test in a `debian:11` container at `:232-240` and the native one at `:245-248`. The claim was true of the old workflow |
| `container-images.yml:15` cites `release.yml:206` for "uploads the binaries" | drifted; `release.yml:206` is an `echo` inside the readelf check. The uploads are at `:250` and `:259` |
| `container-images.yml:219` cites `install.sh:222-228` for reading a version out of a fetched tree | drifted; `install.sh:220-230` is the root/`SUDO_USER` refusal. The version-charset rejection it means is `install.sh:468-474` |
| `distribution.yml:434` cites `.github/ISSUE_TEMPLATE/bug_report.yml:66` for "Ubuntu 20.04" | correct; `bug_report.yml:66` is `- Ubuntu 20.04` |

The `glibc-floor` job is red today and is reporting accurately: the published
`horus-linux-amd64` was built before the zig matrix existed and needs GLIBC_2.39
(`README.md:592-599`). Its own comment says to remove `continue-on-error` once a
release built by the current matrix is published (`distribution.yml:389-392`).
That is a step in the next release, not a cleanup task — until it happens, that
job cannot fail the workflow it is listed in.

---

## What in this document could not be verified from the repository

- Whether `ghcr.io/softmata/horus` holds any image. Both the anonymous GHCR
  token endpoint and `gh api '/orgs/softmata/packages?package_type=container'`
  return 403 from here. Only the absence of a tag-triggered run is verifiable.
- PyPI's own rules about re-uploading a filename. The "one-way door" argument
  above rests entirely on `build-wheels.yml:239-240` omitting `--skip-existing`
  by design.
- That a non-administrator's direct push to `main` is rejected.
  `enforce_admins: false` comes from the API; the consequence for everyone else
  is standard branch-protection behaviour.
- Five statements rest on `gh` API and run-log reads rather than files: the
  branch protection and ruleset settings, the run histories of `release.yml`,
  `build-wheels.yml` and `container-images.yml`, `releases/latest`, the job
  conclusions of run 33830955509, and the log of run 33810476114 (job
  100846077325). Each is reproducible with the command given beside it, but not
  from a checkout alone.
- Nothing here was confirmed by building or testing the workspace. No
  `cargo test`, `cargo check` or `cargo build` was run while writing it.
