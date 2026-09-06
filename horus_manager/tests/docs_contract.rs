//! The Configuration Reference must actually reference the configuration.
//!
//! `horus.toml` accepts 15 top-level tables. The page titled "Configuration
//! Reference" documented four of them:
//!
//! ```text
//! documented: package, dependencies, ignore  (+ enable buried in prose)
//! missing:    workspace, robot, dev-dependencies, sim-dependencies,
//!             hardware, drivers, sim-drivers, scripts, cpp, hooks, network
//! ```
//!
//! `[hardware]` is how a robot declares its drivers and `[network]` is how two
//! robots talk — neither appeared. A user's only way to discover them was to
//! read `manifest.rs`, and a user who guessed a key wrong got no error either,
//! because the manifest silently ignored unknown keys (see `manifest_lint`).
//!
//! ## Scope
//!
//! The docs are a separate repository (`horus-docs`), so these tests skip when
//! there is no checkout to read — either beside this one, or wherever
//! `HORUS_DOCS_DIR` points. The skip is reported, and under `CI` it is not a
//! skip at all but a failure: a contract job that finds no docs has not passed,
//! it has not run, and for months every "live docs" job in
//! `.github/workflows/docs-contract.yml` did exactly that.
//!
//! That rule collides with tier 1 of the same workflow, which is hermetic by
//! definition — it checks out this repository and nothing else — so every test
//! here that opens a page is `#[ignore]`d, the way `docs_manifest.rs` and
//! `docs_examples_cpp.rs` already mark theirs. Tier 1 then runs what it can
//! genuinely run (the skip-or-fail policy, and the wiring guard below) and
//! tier 2 runs everything with `--include-ignored`. Without that split the
//! hermetic job ran eleven docs tests against no docs and failed all eleven,
//! daily, on main.
//!
//! `#[ignore]` is the same lever that hid these tests before, so it gets the
//! guard `install_contract.rs` puts on its network tests:
//! [`the_live_docs_job_runs_the_tests_this_file_ignores`] fails if the live job
//! stops asking for them. Skipping is still never silent on CI.
//!
//! Run: `cargo test -p horus_manager --test docs_contract -- --include-ignored`

use horus_manager::manifest_lint::KNOWN_TOP_LEVEL;
use std::path::{Path, PathBuf};

/// The `horus-docs` checkout, if there is one.
///
/// `HORUS_DOCS_DIR` comes first. Every "live docs" job in
/// `.github/workflows/docs-contract.yml` checks the docs out to
/// `$GITHUB_WORKSPACE/horus-docs` and exports that variable, which is *not*
/// beside this repository — so while this function only accepted a sibling,
/// every one of those jobs hit the `SKIP:` branch and reported success without
/// reading a single page. `docs_cli_contract.rs` and `docs_manifest.rs` already
/// honour the variable ("Without this the 'live docs' CI job silently
/// skipped"); this file never got the fix, so the environment-variable and
/// navigation contracts were enforced on a developer's laptop and nowhere else.
///
/// The directory must actually look like the docs repo. Pointing the variable
/// at an empty or wrong directory falls through to the sibling rather than
/// silently disabling the checks.
fn docs_root() -> Option<PathBuf> {
    if let Ok(dir) = std::env::var("HORUS_DOCS_DIR") {
        let p = PathBuf::from(dir);
        if p.join("content/docs").is_dir() {
            return Some(p);
        }
        eprintln!(
            "HORUS_DOCS_DIR={} has no content/docs — ignoring it",
            p.display()
        );
    }
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()? // horus/
        .parent()? // softmata/
        .join("horus-docs");
    root.join("content/docs").is_dir().then_some(root)
}

/// `docs_root()`, but a missing checkout is a hard failure on CI.
///
/// Skipping is right on a developer's machine with only one repository cloned.
/// It is never right on a runner: the "live docs" jobs exist to check the docs,
/// so a run that finds none has not passed, it has not run. That distinction
/// was invisible for as long as `docs_root()` ignored `HORUS_DOCS_DIR` — every
/// live job printed `SKIP:` and went green — and would become invisible again
/// the moment someone dropped the `env:` block from a workflow step. `CI` is
/// set by GitHub Actions, GitLab CI, CircleCI and Buildkite alike.
fn docs_root_or_skip() -> Option<PathBuf> {
    resolve_docs(docs_root(), std::env::var_os("CI").is_some())
}

/// The skip-or-fail decision, separated from the filesystem and the environment
/// so it can be tested. Whether a missing checkout is tolerable is not a fact
/// about this machine; it is a fact about who is running.
fn resolve_docs(found: Option<PathBuf>, on_ci: bool) -> Option<PathBuf> {
    if found.is_some() {
        return found;
    }
    assert!(
        !on_ci,
        "running under CI with no horus-docs checkout. Check the docs out and \
         set HORUS_DOCS_DIR, or drop this job — a docs contract that silently \
         skips is worse than one that does not exist, because the workflow \
         still reports it as passing."
    );
    eprintln!("SKIP: no horus-docs checkout (set HORUS_DOCS_DIR or clone it beside horus)");
    None
}

#[test]
fn a_missing_checkout_skips_locally() {
    assert!(
        resolve_docs(None, false).is_none(),
        "a developer with one repository cloned should not be failed"
    );
}

#[test]
#[should_panic(expected = "running under CI with no horus-docs checkout")]
fn a_missing_checkout_is_fatal_under_ci() {
    // Every "live docs" step in .github/workflows/docs-contract.yml exists to
    // read the docs. One that finds none has not passed; it has not run. This
    // is the state that reported success for months.
    resolve_docs(None, true);
}

// ─── The workflow wiring ─────────────────────────────────────────────────────
//
// The two tests above are the whole reason a docs test cannot simply run in the
// hermetic tier, and the `#[ignore]` that keeps it out of there is the reason
// the next test exists.

/// The body of one job in a workflow file.
///
/// Jobs are the two-space-indented keys under `jobs:`; the block runs to the
/// next one. Same shape as `workflow_job` in `examples_contract.rs` — copied
/// rather than shared because integration tests are separate binaries and this
/// is nine lines.
fn workflow_job(workflow: &str, job: &str) -> String {
    let header = format!("  {job}:");
    let mut body = Vec::new();
    let mut inside = false;
    for line in workflow.lines() {
        if line.trim_end() == header {
            inside = true;
            continue;
        }
        if inside {
            let next_key = line.starts_with("  ")
                && !line.starts_with("   ")
                && !line.trim_start().starts_with('#');
            if next_key {
                break;
            }
            body.push(line);
        }
    }
    assert!(!body.is_empty(), "no job `{job}` in docs-contract.yml");
    body.join("\n")
}

/// The `run:` lines in one job that invoke this test binary.
///
/// Comments are dropped: the workflow discusses `--test docs_contract` in prose
/// in both jobs, including one comment quoting an `--ignored` invocation that
/// no longer exists.
fn docs_contract_steps(job: &str) -> Vec<&str> {
    job.lines()
        .map(str::trim)
        .filter(|l| !l.starts_with('#') && l.contains("--test docs_contract"))
        .collect()
}

/// Ignoring the docs tests is only acceptable if something still runs them.
///
/// This is the guarantee `docs_root_or_skip` makes, restated at the level the
/// `#[ignore]` moved it to. A missing checkout still cannot pass silently — but
/// a *missing invocation* now can, because an ignored test that nobody asks for
/// is reported by cargo as `ok. 11 ignored` and by the job as green. That is
/// the exact failure mode the module comment describes for `SKIP:`, one layer
/// out, so it gets a check rather than a promise.
#[test]
fn the_live_docs_job_runs_the_tests_this_file_ignores() {
    let workflow = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager has a parent")
        .join(".github/workflows/docs-contract.yml");
    let wf = std::fs::read_to_string(&workflow)
        .unwrap_or_else(|e| panic!("{} must be readable: {e}", workflow.display()));

    let live = workflow_job(&wf, "contract-live");
    let live_steps = docs_contract_steps(&live);
    assert!(
        !live_steps.is_empty(),
        "contract-live no longer runs docs_contract at all, so nothing reads \
         the pages: the tests here are `#[ignore]`d and the hermetic job has no \
         checkout to read."
    );
    // `--include-ignored`, not `--ignored`: the skip-policy tests above are not
    // ignored and are worth running against a real checkout too.
    assert!(
        live_steps.iter().any(|l| l.contains("--include-ignored")),
        "contract-live runs docs_contract without `--include-ignored`, so every \
         page-reading test here is skipped and the job reports success for \
         checking nothing:\n  {}",
        live_steps.join("\n  ")
    );
    assert!(
        live.contains("HORUS_DOCS_DIR"),
        "contract-live does not point HORUS_DOCS_DIR at its horus-docs \
         checkout, so the docs tests would fail on the missing-checkout \
         assertion instead of reading the pages"
    );

    // The other direction. The hermetic tier checks out this repository and
    // nothing else, so asking it for the ignored tests puts it straight back
    // into failing all of them by design.
    let hermetic = workflow_job(&wf, "contract-hermetic");
    let hermetic_steps = docs_contract_steps(&hermetic);
    assert!(
        !hermetic_steps.is_empty(),
        "contract-hermetic no longer runs docs_contract, so this guard and the \
         skip-policy tests do not run on a PR"
    );
    for step in &hermetic_steps {
        assert!(
            !step.contains("ignored"),
            "contract-hermetic asks docs_contract for ignored tests, but it \
             checks out no docs — every one of them fails on the \
             missing-checkout assertion:\n  {step}"
        );
    }
}

fn config_reference() -> Option<String> {
    let path = docs_root_or_skip()?.join("content/docs/package-management/configuration.mdx");
    std::fs::read_to_string(path).ok()
}

/// Every table the manifest accepts must appear in the reference.
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]
fn the_configuration_reference_covers_every_manifest_table() {
    // `config_reference` goes through `docs_root_or_skip`, so a missing
    // checkout has already failed the test if this is CI.
    let Some(doc) = config_reference() else {
        return;
    };

    let mut undocumented = Vec::new();
    for key in KNOWN_TOP_LEVEL {
        // `[key]` for a table; `key = ` for the one bare key (`enable`).
        //
        // The fallback used to be `contains("`key`")` — the name in backticks
        // anywhere on the page. That is satisfied by any passing mention, and
        // one existed: a language-detection table lists `rust` as a *language*,
        // which made the `[rust]` config section look documented while it was
        // absent entirely. Deleting the whole section still passed this test.
        let documented = doc.contains(&format!("[{key}]")) || doc.contains(&format!("{key} = "));
        if !documented {
            undocumented.push(*key);
        }
    }

    assert!(
        undocumented.is_empty(),
        "horus.toml accepts these keys but the Configuration Reference never \
         mentions them: {undocumented:?}\n\n\
         An undocumented key is one a user can only find by reading the source. \
         Add a section to content/docs/package-management/configuration.mdx, or \
         remove the key from the manifest."
    );
}

/// The reference should show each table in use, not merely name it in a list.
/// A key that only appears inside the Quick Reference block is not documented.
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]
fn each_table_gets_a_section_not_just_a_mention() {
    // `config_reference` goes through `docs_root_or_skip`, so a missing
    // checkout has already failed the test if this is CI.
    let Some(doc) = config_reference() else {
        return;
    };

    // Headings look like `## Hooks` / `### \`[hardware]\` (Optional)`.
    let headings: Vec<&str> = doc
        .lines()
        .filter(|l| l.trim_start().starts_with('#'))
        .collect();
    let heading_text = headings.join("\n").to_lowercase();

    // A handful of keys are documented under a shared heading, which is fine —
    // `[drivers]`/`[sim-drivers]` belong with `[hardware]`, and the three
    // dependency tables belong together. Those are listed here explicitly so
    // that adding a *new* key cannot quietly inherit the exemption.
    let grouped: &[(&str, &str)] = &[
        ("drivers", "hardware"),
        ("sim-drivers", "hardware"),
        ("dev-dependencies", "dependencies"),
        ("sim-dependencies", "dependencies"),
    ];

    let mut sectionless = Vec::new();
    for key in KNOWN_TOP_LEVEL {
        let own = heading_text.contains(&key.to_lowercase());
        let under_group = grouped
            .iter()
            .any(|(k, parent)| k == key && heading_text.contains(parent));
        if !own && !under_group {
            sectionless.push(*key);
        }
    }

    assert!(
        sectionless.is_empty(),
        "these keys are mentioned but have no section of their own: \
         {sectionless:?}\n\n\
         Being listed in an example is not documentation — a reader needs to \
         know what the fields mean."
    );
}

/// The lifecycle hooks are the most recently added surface and the easiest to
/// ship undocumented, so they get their own check.
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]
fn every_hook_phase_is_documented() {
    // `config_reference` goes through `docs_root_or_skip`, so a missing
    // checkout has already failed the test if this is CI.
    let Some(doc) = config_reference() else {
        return;
    };

    let mut missing = Vec::new();
    for hook in horus_manager::manifest_lint::KNOWN_HOOKS {
        if !doc.contains(hook) {
            missing.push(*hook);
        }
    }

    assert!(
        missing.is_empty(),
        "these hooks exist but are undocumented: {missing:?}"
    );
}

/// A guard on the guard: if the reference file moves or is renamed, the tests
/// above would skip forever and report nothing. This fails instead.
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]
fn the_configuration_reference_is_where_we_think_it_is() {
    let Some(root) = docs_root_or_skip() else {
        return;
    };

    let path = root.join("content/docs/package-management/configuration.mdx");
    assert!(
        path.is_file(),
        "the docs repo is present but {} is missing. If the reference moved, \
         update docs_contract.rs — otherwise the coverage tests above silently \
         stop checking anything.",
        path.display()
    );
}

/// A command HORUS tells the user to run must be documented.
///
/// `horus doctor` ended its real-time check with "run `horus setup-rt` for
/// ±20μs". `setup-rt` appeared **zero** times in the entire docs repository, so
/// the one place a user is told about it is a warning line, with nothing to
/// read about what it will do to their machine — and it installs a kernel
/// package and edits `/etc/security/limits.conf`.
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]
fn commands_the_cli_suggests_are_documented() {
    let Some(root) = docs_root_or_skip() else {
        return;
    };

    // Commands the CLI emits to the user as advice. Add to this list when a new
    // suggestion is introduced; the test then requires it to be documented.
    let suggested = ["setup-rt", "doctor", "check", "lint", "fmt", "doc"];

    let mut corpus = String::new();
    let mut stack = vec![root.clone()];
    while let Some(dir) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                stack.push(path);
            } else if path.extension().is_some_and(|e| e == "mdx" || e == "md") {
                if let Ok(text) = std::fs::read_to_string(&path) {
                    corpus.push_str(&text);
                }
            }
        }
    }

    let missing: Vec<&str> = suggested
        .iter()
        .copied()
        .filter(|c| !corpus.contains(&format!("horus {c}")))
        .collect();

    assert!(
        missing.is_empty(),
        "the CLI tells users to run these, but the documentation never mentions \
         them: {missing:?}\n\nA command that modifies the user's system needs a \
         page they can read first."
    );
}

/// Every documentation page must be reachable from the navigation.
///
/// 53 of 120 pages were not, including the entire C++ surface: all 25 `cpp/`
/// API pages and all 12 tutorials — every one of which is a C++ tutorial. There
/// was no "C++" section in the sidebar at all, next to the "Rust" and "Python"
/// ones. For a framework that lists C++ as a first-class language, its whole
/// documentation tree existed but could not be navigated to.
///
/// Also asserts the two lists stay identical. `PrevNextNav.tsx` says in a
/// comment that it "must match DocsSidebar.tsx", and nothing checked it.
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]
fn every_docs_page_is_reachable_from_the_navigation() {
    let Some(root) = docs_root_or_skip() else {
        return;
    };
    // This used to be `root.parent().parent().join("components")` — two levels
    // *above* the docs repo, i.e. `/home/<user>/components`, which does not
    // exist anywhere. The test therefore hit the skip branch on every machine
    // and every runner and reported `ok` while 24 pages were orphaned. It is a
    // hard failure now: if the docs repo is present, its components are too, and
    // a missing one means the layout moved rather than that there is nothing to
    // check.
    let components = root.join("components");
    assert!(
        components.is_dir(),
        "{} is missing. The docs repo is checked out, so this is not a \
         'nothing to check' state — the navigation moved and this guard needs \
         updating rather than skipping.",
        components.display()
    );

    let sidebar = std::fs::read_to_string(components.join("DocsSidebar.tsx"))
        .expect("DocsSidebar.tsx must exist");
    // Not read, only required to exist: if it is gone, every page's prev/next
    // links are gone with it and `prev_next_navigation_is_derived_from_the_sidebar`
    // is the test that would say so.
    assert!(
        components.join("PrevNextNav.tsx").is_file(),
        "PrevNextNav.tsx must exist"
    );

    // Walk `content/docs`, not the repository root. Routes are relative to it —
    // `content/docs/tutorials/01-x.mdx` is served at `/tutorials/01-x` — so
    // stripping the repo root instead produced `/content/docs/tutorials/01-x`,
    // which matches nothing in the sidebar. Had the path above been correct,
    // this would have reported all 145 pages as orphaned and been reverted as
    // noise. Rooting the walk here also keeps it out of `node_modules/` and
    // `.next/`, which hold thousands of unrelated markdown files.
    let content = root.join("content/docs");
    let mut unreachable = Vec::new();
    let mut pages = 0usize;
    let mut stack = vec![content.clone()];
    while let Some(dir) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                stack.push(path);
                continue;
            }
            if !path.extension().is_some_and(|e| e == "mdx") {
                continue;
            }
            let rel = path
                .strip_prefix(&content)
                .unwrap_or(&path)
                .to_string_lossy()
                .replace(".mdx", "");

            // A redirect stub is deliberately not in the navigation.
            if let Ok(text) = std::fs::read_to_string(&path) {
                if text.contains("Redirect stub") {
                    continue;
                }
            }

            let href = format!("/{rel}");
            // `foo/index` is reached as `/foo`.
            let parent = rel.strip_suffix("/index").map(|p| format!("/{p}"));

            let linked = sidebar.contains(&format!("\"{href}\""))
                || parent
                    .as_deref()
                    .is_some_and(|p| sidebar.contains(&format!("\"{p}\"")));

            pages += 1;
            if !linked {
                unreachable.push(rel);
            }
        }
    }

    // A walk that finds nothing passes trivially. It found nothing for months.
    assert!(
        pages > 100,
        "only {pages} documentation pages were scanned — the walk is broken and \
         this test is vacuous"
    );

    unreachable.sort();
    assert!(
        unreachable.is_empty(),
        "{} documentation page(s) exist but nothing links to them:\n  {}\n\n\
         Add them to `sections` in components/DocsSidebar.tsx. PrevNextNav \
         derives its order from that list, so there is nothing to update \
         there.",
        unreachable.len(),
        unreachable.join("\n  ")
    );

    // A second assertion used to live here: extract `href: "..."` from both
    // files and require the two sets to be equal. It was written when
    // PrevNextNav held a hand-copied list. PrevNextNav now *derives* its order
    // from `sections` — `prev_next_navigation_is_derived_from_the_sidebar`
    // below asserts it contains zero literal hrefs — so the extraction returns
    // an empty set for it and the equality can never hold again. It never fired
    // only because the orphan assertion above it was unreachable; fixing that
    // would have turned this into a guaranteed failure for the wrong reason.
    // Derivation is the stronger guarantee, so the check is gone rather than
    // adjusted.
}

/// The tutorial series must exist in every language HORUS ships.
///
/// `docs.config.ts` used to state this as a rule for the `tutorials/`
/// directory — "Numbered sequence: 01-sensor -> 02-motor -> ...", "Both Rust
/// and Python versions of each tutorial". An unrelated merge deleted that file,
/// so the rule stopped being stated at all; nothing had ever enforced it. It
/// was not being met either: tutorials 04-10 existed only for C++ while the
/// front page lists Rust, Python and C++ as equals, so a Rust reader's guided
/// path ended at tutorial 3.
///
/// Encoded as a test rather than as prose in a config file because prose in a
/// config file is what was there before.
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]
fn every_tutorial_exists_in_every_language() {
    let Some(root) = docs_root_or_skip() else {
        return;
    };
    let dir = root.join("content/docs/tutorials");
    assert!(dir.is_dir(), "{} must exist", dir.display());

    // Languages with a first-class quick start. Adding one here makes the whole
    // tutorial series a requirement for it, which is the point.
    const LANGUAGES: [&str; 3] = ["rust", "python", "cpp"];

    // stem ("04-custom-messages") -> languages present
    let mut series: std::collections::BTreeMap<String, std::collections::BTreeSet<String>> =
        std::collections::BTreeMap::new();
    let mut unsuffixed = Vec::new();

    for entry in std::fs::read_dir(&dir)
        .expect("tutorials/ must be readable")
        .flatten()
    {
        let path = entry.path();
        if !path.extension().is_some_and(|e| e == "mdx") {
            continue;
        }
        let name = path
            .file_stem()
            .unwrap_or_default()
            .to_string_lossy()
            .to_string();
        if name == "index" {
            continue;
        }
        match LANGUAGES.iter().find(|l| name.ends_with(&format!("-{l}"))) {
            Some(lang) => {
                let stem = name[..name.len() - lang.len() - 1].to_string();
                series.entry(stem).or_default().insert((*lang).to_string());
            }
            // A page with no language suffix is how the series drifted before:
            // `04-custom-messages.mdx` was C++ content under a neutral name, so
            // nothing looked missing.
            None => unsuffixed.push(name),
        }
    }

    assert!(
        unsuffixed.is_empty(),
        "tutorial pages without a -rust/-python/-cpp suffix: {unsuffixed:?}\n\n\
         Every tutorial is language-specific. A neutral filename hides which \
         language it is written for and hides that the others are missing."
    );
    assert!(
        series.len() >= 10,
        "found only {} tutorials — the scan is broken and this test is vacuous",
        series.len()
    );

    let mut incomplete = Vec::new();
    for (stem, langs) in &series {
        let missing: Vec<&str> = LANGUAGES
            .iter()
            .copied()
            .filter(|l| !langs.contains(*l))
            .collect();
        if !missing.is_empty() {
            incomplete.push(format!("{stem}: missing {}", missing.join(", ")));
        }
    }
    assert!(
        incomplete.is_empty(),
        "the tutorial series is not complete in every language:\n  {}\n\n\
         HORUS presents Rust, Python and C++ as equals. A reader who picked one \
         of them and found the guided path stops at tutorial 3 has been told \
         their language is second class.",
        incomplete.join("\n  ")
    );

    // The numbered prefixes must be a contiguous run starting at 01: a gap
    // means a tutorial referenced as "the next one" does not exist.
    let mut numbers: Vec<u32> = series
        .keys()
        .filter_map(|s| s.split('-').next())
        .filter_map(|n| n.parse().ok())
        .collect();
    numbers.sort_unstable();
    numbers.dedup();
    assert!(
        !numbers.is_empty(),
        "no numbered tutorials found — the prefix parse is broken"
    );
    let expected: Vec<u32> = (1..=numbers.len() as u32).collect();
    assert_eq!(
        numbers, expected,
        "the numbered tutorial sequence has a gap or does not start at 01. \
         Each tutorial builds on the one before it, so a missing number breaks \
         the path rather than shortening it."
    );
}

// ─── Environment variables ───────────────────────────────────────────────────

/// A `HORUS_*` name the shipped product reads, or writes into a process the
/// user can observe, together with where it was found.
#[derive(Debug, Default)]
struct EnvSurface {
    /// name -> first two sites, for the failure message.
    sites: std::collections::BTreeMap<String, Vec<String>>,
}

impl EnvSurface {
    fn note(&mut self, name: &str, site: String) {
        let e = self.sites.entry(name.to_string()).or_default();
        if e.len() < 2 && !e.contains(&site) {
            e.push(site);
        }
    }
}

/// Byte ranges covered by `#[cfg(test)] mod ... { }` / `#[cfg(test)] fn ... { }`.
///
/// Needed because `env::set_var("HORUS_PARAM_TEST_SPEED", ..)` in
/// `horus_core/src/params.rs` is a unit test, not a knob. Truncating each file
/// at its first `#[cfg(test)]` was the cheap alternative and it is wrong: eight
/// files in this workspace declare a test module partway through and keep going
/// (`plugins/executor.rs` at 8% of the file), which would have hidden
/// `HORUS_PLUGIN`, `HORUS_VERSION`, `HORUS_NET_DENY_EXPORT` and
/// `HORUS_ESTOP_STDERR_CHILD` — every one of them real.
fn test_regions(src: &str) -> Vec<(usize, usize)> {
    let bytes = src.as_bytes();
    let mut out = Vec::new();
    for (attr, _) in src.match_indices("#[cfg(test)]") {
        let Some(open_rel) = src[attr..].find('{') else {
            continue;
        };
        let open = attr + open_rel;
        // Only a `mod`/`fn` block, not e.g. `#[cfg(test)] use x::{a, b};`.
        let head = &src[attr..open];
        if !head.contains("mod ") && !head.contains("fn ") {
            continue;
        }
        let mut depth = 0usize;
        let mut i = open;
        while i < bytes.len() {
            match bytes[i] {
                b'"' => {
                    // Skip the string literal; a lone `{` inside one would
                    // otherwise unbalance the count.
                    i += 1;
                    while i < bytes.len() && bytes[i] != b'"' {
                        i += if bytes[i] == b'\\' { 2 } else { 1 };
                    }
                }
                b'/' if bytes.get(i + 1) == Some(&b'/') => {
                    while i < bytes.len() && bytes[i] != b'\n' {
                        i += 1;
                    }
                    continue;
                }
                b'{' => depth += 1,
                b'}' => {
                    depth -= 1;
                    if depth == 0 {
                        break;
                    }
                }
                _ => {}
            }
            i += 1;
        }
        out.push((attr, i));
    }
    out
}

/// Pull the `HORUS_*` argument out of every `pat("HORUS_…"` in `src`, skipping
/// matches inside a test module.
fn env_names_matching(
    src: &str,
    pat: &str,
    skip: &[(usize, usize)],
    out: &mut Vec<(String, usize)>,
) {
    for (idx, _) in src.match_indices(pat) {
        let after = &src[idx + pat.len()..];
        // Tolerate whitespace and the opening paren between the call and the
        // literal: `env::var ( "HORUS_X" )`.
        let Some(q) = after.find('"') else { continue };
        if after[..q].contains(|c: char| !c.is_whitespace() && c != '(' && c != ',') {
            continue;
        }
        let rest = &after[q + 1..];
        let Some(close) = rest.find('"') else {
            continue;
        };
        let name = &rest[..close];
        if !name.starts_with("HORUS_")
            || !name
                .chars()
                .all(|c| c.is_ascii_uppercase() || c.is_ascii_digit() || c == '_')
        {
            continue;
        }
        if skip.iter().any(|(a, b)| idx >= *a && idx <= *b) {
            continue;
        }
        out.push((name.to_string(), idx));
    }
}

/// Names reached through a `const`, rather than written as a literal at the
/// call site.
///
/// The literal scanners above see `env::var("HORUS_X")` and nothing else, so
/// the idiomatic way to name a variable —
///
/// ```ignore
/// pub const SEND_RETRY_BUDGET_ENV: &str = "HORUS_SEND_RETRY_BUDGET_US";
/// std::env::var(SEND_RETRY_BUDGET_ENV)
/// ```
///
/// — was invisible to them. That is not a hypothetical: `HORUS_WIN_REALTIME_CLASS`
/// and `HORUS_SEND_RETRY_BUDGET_US` were both added behind a `const`, both are
/// operator-facing knobs, and both passed this test while absent from the page.
/// A guard blind to the tidier of two spellings mostly proves that people write
/// the untidy one.
///
/// Declaration and use must be in the same file. A `const` exported and read
/// from another crate would still be missed; every current site is local, and
/// resolving across files needs a symbol table this test has no business
/// growing.
fn const_env_names(src: &str, skip: &[(usize, usize)], out: &mut Vec<(String, usize)>) {
    // `const NAME: &str = "HORUS_..."` / `static NAME: &str = "HORUS_..."`.
    let mut consts: Vec<(String, String)> = Vec::new();
    for line in src.lines() {
        let t = line.trim_start();
        let t = t
            .strip_prefix("pub(crate) ")
            .or_else(|| t.strip_prefix("pub(super) "))
            .unwrap_or(t);
        let t = t.strip_prefix("pub ").unwrap_or(t);
        let Some(rest) = t
            .strip_prefix("const ")
            .or_else(|| t.strip_prefix("static "))
        else {
            continue;
        };
        let Some((ident, tail)) = rest.split_once(':') else {
            continue;
        };
        let ident = ident.trim();
        if ident.is_empty()
            || !ident
                .chars()
                .all(|c| c.is_ascii_uppercase() || c.is_ascii_digit() || c == '_')
        {
            continue;
        }
        let Some((ty, value)) = tail.split_once('=') else {
            continue;
        };
        if !ty.contains("str") {
            continue;
        }
        let value = value.trim();
        let Some(v) = value.strip_prefix('"') else {
            continue;
        };
        let Some(end) = v.find('"') else { continue };
        let name = &v[..end];
        if name.starts_with("HORUS_")
            && name
                .chars()
                .all(|c| c.is_ascii_uppercase() || c.is_ascii_digit() || c == '_')
        {
            consts.push((ident.to_string(), name.to_string()));
        }
    }

    for (ident, name) in consts {
        for pat in [
            "env::var(",
            "env::var_os(",
            "env::set_var(",
            ".env(",
            "env.insert(",
        ] {
            for (idx, _) in src.match_indices(pat) {
                // Tolerate whitespace between the call and the identifier, the
                // same way `env_names_matching` tolerates it before the quote:
                // `env::var( SEND_RETRY_BUDGET_ENV )` is the same call.
                let after = src[idx + pat.len()..].trim_start();
                let arg_end = after
                    .find(|c: char| !(c.is_alphanumeric() || c == '_'))
                    .unwrap_or(after.len());
                if after[..arg_end] != ident {
                    continue;
                }
                if skip.iter().any(|(a, b)| idx >= *a && idx <= *b) {
                    continue;
                }
                out.push((name.clone(), idx));
            }
        }
    }
}

/// Everything a user can observe: names HORUS reads, and names HORUS sets in a
/// child process's environment.
///
/// Scans four languages, not one. The previous version read only `*.rs` files
/// whose path contained `/src/`, and only `env::var*(..)` within them, so it
/// was blind to:
///
/// * `install.sh`, which reads `HORUS_BUILD_FROM_SOURCE` and prints
///   "Build from source instead: `HORUS_BUILD_FROM_SOURCE=1 $0`" at the reader;
/// * every variable HORUS *sets* — `HORUS_NODE_FILE` is how the wrapper
///   generated for `horus run <file>.py` finds the file it is meant to run, and
///   `HORUS_DRIVER_NAME` is injected into every user-written exec driver;
/// * C++ and Python readers.
fn environment_surface(repo: &Path) -> EnvSurface {
    let mut surface = EnvSurface::default();
    let mut stack = vec![repo.to_path_buf()];
    while let Some(dir) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            let name = entry.file_name().to_string_lossy().to_string();
            if path.is_dir() {
                if name != "target" && !name.starts_with('.') && name != "node_modules" {
                    stack.push(path);
                }
                continue;
            }
            let rel = path
                .strip_prefix(repo)
                .unwrap_or(&path)
                .to_string_lossy()
                .replace('\\', "/");
            // A name that only a test reads is not a user-facing knob, and the
            // test suite writes about the very variables it checks.
            let in_tests = rel.starts_with("tests/")
                || rel.contains("/tests/")
                || rel.ends_with("/tests.rs")
                || rel.contains("/benches/");
            let Ok(body) = std::fs::read_to_string(&path) else {
                continue;
            };
            let ext = path
                .extension()
                .unwrap_or_default()
                .to_string_lossy()
                .to_string();
            let mut hits: Vec<(String, usize)> = Vec::new();

            match ext.as_str() {
                "rs" if !in_tests && rel.contains("/src/") => {
                    let skip = test_regions(&body);
                    // Read. `env::var` is a prefix of `env::vars`, which takes
                    // no argument — matching it picks up whatever string comes
                    // next in the file, which is how a `strip_prefix(..)` twelve
                    // lines below an `env::vars()` call became a "variable".
                    // `env_names_matching` rejects it: the text between the call
                    // and the quote is not just whitespace and punctuation.
                    env_names_matching(&body, "env::var(", &skip, &mut hits);
                    env_names_matching(&body, "env::var_os(", &skip, &mut hits);
                    // Set, for this process or for a child.
                    env_names_matching(&body, "env::set_var(", &skip, &mut hits);
                    env_names_matching(&body, ".env(", &skip, &mut hits);
                    env_names_matching(&body, "env.insert(", &skip, &mut hits);
                    // Named through a `const`, which the literal scanners above
                    // cannot see.
                    const_env_names(&body, &skip, &mut hits);
                }
                "cpp" | "cc" | "hpp" | "h" if !in_tests => {
                    env_names_matching(&body, "getenv(", &[], &mut hits);
                    env_names_matching(&body, "secure_getenv(", &[], &mut hits);
                }
                "py" if !in_tests => {
                    env_names_matching(&body, "os.environ.get(", &[], &mut hits);
                    env_names_matching(&body, "os.environ[", &[], &mut hits);
                    env_names_matching(&body, "os.getenv(", &[], &mut hits);
                }
                _ => {}
            }

            // Shipped shell. A script assigns plenty of names for its own use
            // (`HORUS_CACHE="$HOME/.horus/cache"`); only a name it never
            // assigns is one it expects from the environment.
            if name == "install.sh" || name == "uninstall.sh" {
                let assigned: std::collections::BTreeSet<String> = body
                    .lines()
                    .filter_map(|l| {
                        let l = l.trim_start().trim_start_matches("export ");
                        let (lhs, _) = l.split_once('=')?;
                        lhs.starts_with("HORUS_").then(|| lhs.to_string())
                    })
                    .collect();
                for part in body.split('$').skip(1) {
                    let part = part.trim_start_matches('{');
                    let end = part
                        .find(|c: char| !(c.is_ascii_uppercase() || c.is_ascii_digit() || c == '_'))
                        .unwrap_or(part.len());
                    let candidate = &part[..end];
                    if candidate.starts_with("HORUS_")
                        && candidate.len() > "HORUS_".len()
                        && !assigned.contains(candidate)
                    {
                        hits.push((candidate.to_string(), 0));
                    }
                }
            }

            for (n, _) in hits {
                surface.note(&n, rel.clone());
            }
        }
    }
    surface
}

/// Every environment variable must appear in the reference page, by name.
///
/// Two separate ways this used to pass while variables were missing:
///
/// 1. The membership test was `page.contains(name)`, a substring match. A name
///    that is a prefix of a documented one satisfied it without appearing:
///    `HORUS_NET` was "documented" by `HORUS_NET_PORT`, and `HORUS_SOURCE` by
///    `HORUS_SOURCE_DIR`. Both are real and neither was on the page — and
///    `horus doctor` prints "use `--net` or `HORUS_NET=1` to enable" at the
///    user. The check is now name-exact.
/// 2. The scan covered one language and one direction. See
///    [`environment_surface`].
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]
fn every_environment_variable_is_in_the_reference_page() {
    let Some(root) = docs_root_or_skip() else {
        return;
    };
    let page = root.join("content/docs/development/environment-variables.mdx");
    let Ok(text) = std::fs::read_to_string(&page) else {
        panic!(
            "the environment variable reference must exist at {}",
            page.display()
        );
    };

    let repo = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager must have a parent")
        .to_path_buf();
    let surface = environment_surface(&repo);

    assert!(
        surface.sites.len() > 40,
        "found only {} environment variables — the scan is broken and this test \
         is vacuous",
        surface.sites.len()
    );
    // Each scanner must find something. Losing one silently is how
    // `install.sh` and the whole "set by HORUS" direction went unchecked,
    // and how every `const`-named variable went unchecked after that.
    for (what, probe) in [
        ("rust reads", "HORUS_NET_PORT"),
        ("rust sets", "HORUS_NODE_FILE"),
        ("shell", "HORUS_BUILD_FROM_SOURCE"),
        ("rust const", "HORUS_SEND_RETRY_BUDGET_US"),
    ] {
        assert!(
            surface.sites.contains_key(probe),
            "the {what} scanner found nothing: {probe} is missing from the \
             scan, so this test would pass no matter what the page said"
        );
    }

    let missing: Vec<String> = surface
        .sites
        .iter()
        .filter(|(name, _)| !mentions_exactly(&text, name))
        .map(|(name, sites)| format!("{name}  ({})", sites.join(", ")))
        .collect();

    assert!(
        missing.is_empty(),
        "these are read from, or written into, a process environment but are \
         absent from the reference page:\n  {}\n\n\
         Add them to content/docs/development/environment-variables.mdx. A \
         variable HORUS sets for your node is as much a contract as one it \
         reads.",
        missing.join("\n  ")
    );
}

/// Does `text` name `var`, as opposed to merely containing its letters?
///
/// `HORUS_NET_PORT`.contains("HORUS_NET") is true, which is why the substring
/// version of this check reported full coverage.
fn mentions_exactly(text: &str, var: &str) -> bool {
    let boundary = |c: Option<char>| !c.is_some_and(|c| c.is_ascii_alphanumeric() || c == '_');
    text.match_indices(var).any(|(i, _)| {
        boundary(text[..i].chars().next_back()) && boundary(text[i + var.len()..].chars().next())
    })
}

// `every_environment_variable_is_documented` used to live here: the same scan,
// asserted against every .mdx in the repository rather than against the
// reference page. It carried no `#[test]` of its own — a second `#[test]` had
// been stacked on the function above instead, so that one ran twice and this
// one never ran at all (`warning: function ... is never used`, reproduced on
// every build).
//
// It is deleted rather than wired up because it cannot fail unless the test
// above fails first: the reference page is one of the files it concatenated, so
// "absent from the whole corpus" is strictly weaker than "absent from the
// page". Running it would have added a second failure to every real breakage
// and caught nothing on its own.

// ─── Duplication between concepts/ and the language references ───────────────

/// Headings that say nothing about a page's subject.
///
/// Two pages both having a "See Also" is not duplication; two pages both having
/// a "BatteryState" is.
const BOILERPLATE_HEADINGS: &[&str] = &[
    "see also",
    "next steps",
    "overview",
    "summary",
    "troubleshooting",
    "examples",
    "example",
    "best practices",
    "performance",
    "when to use",
    "requirements",
    "installation",
    "api reference",
    "quick reference",
    "tips and tricks",
    "common questions",
    "usage",
    "availability",
    "notes",
    "key takeaways",
    "prerequisites",
];

/// Markdown headings in `text`, excluding fenced code and boilerplate.
///
/// Fences must be excluded or every C++ page "shares" `include <cstdio>` with
/// every other one: `#include` starts with `#`.
fn subject_headings(text: &str) -> std::collections::BTreeSet<String> {
    let mut out = std::collections::BTreeSet::new();
    let mut in_fence = false;
    for line in text.lines() {
        let t = line.trim();
        if t.starts_with("```") {
            in_fence = !in_fence;
            continue;
        }
        if in_fence || !t.starts_with('#') {
            continue;
        }
        let h = t.trim_start_matches('#').trim();
        if h.is_empty() || BOILERPLATE_HEADINGS.contains(&h.to_lowercase().as_str()) {
            continue;
        }
        out.insert(h.to_string());
    }
    out
}

/// `concepts/` must not restate the per-language references.
///
/// The directory exists to hold the one language-neutral explanation of each
/// idea; the `rust/`, `python/` and `cpp/` trees hold the per-language detail.
/// When a concepts page grows its own copy of the detail, the copies drift, and
/// the reader has no way to tell which one is current. This had happened at
/// scale and the drift was already load-bearing:
///
/// * `concepts/message-types.mdx` was a full message catalogue — 39 of its
///   headings named an individual message with its own field table. It shared
///   39 headings with `rust/api/*messages*` and 28 with the Python message
///   library. `BatteryState.charge` read "Remaining charge in Ah" there and
///   "Charge in amp-hours (NaN if unknown)" in the Rust reference; the source
///   defaults the field to `f32::NAN`, so the concepts copy was the wrong one.
/// * `concepts/node-macro.mdx` shared 9 headings with `rust/api/macros.mdx`,
///   including two identical compiler-error headings — and disagreed on what
///   the macro does with a snake_case node name.
/// * `concepts/communication-overview.mdx` carried a second Actions guide
///   beside `concepts/actions.mdx`, 6 shared headings, duplication inside
///   `concepts/` itself.
///
/// The threshold is deliberately loose: two pages about the same subject will
/// share a heading or two, and the failure this catches is a page growing back
/// a whole second copy of another one.
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]
fn concepts_pages_do_not_restate_the_language_references() {
    let Some(root) = docs_root_or_skip() else {
        return;
    };
    let content = root.join("content/docs");

    let mut pages: Vec<(String, std::collections::BTreeSet<String>)> = Vec::new();
    let mut stack = vec![content.clone()];
    while let Some(dir) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                stack.push(path);
                continue;
            }
            if !path.extension().is_some_and(|e| e == "mdx") {
                continue;
            }
            let Ok(text) = std::fs::read_to_string(&path) else {
                continue;
            };
            let rel = path
                .strip_prefix(&content)
                .unwrap_or(&path)
                .to_string_lossy()
                .replace('\\', "/");
            pages.push((rel, subject_headings(&text)));
        }
    }
    pages.sort();

    let concepts: Vec<&(String, std::collections::BTreeSet<String>)> = pages
        .iter()
        .filter(|(p, _)| p.starts_with("concepts/"))
        .collect();

    // Vacuity guards: an empty walk, or a heading parser that returns nothing,
    // would make every comparison trivially pass.
    assert!(
        concepts.len() >= 10,
        "found only {} pages under concepts/ — the walk is broken",
        concepts.len()
    );
    let total: usize = concepts.iter().map(|(_, h)| h.len()).sum();
    assert!(
        total > 200,
        "parsed only {total} subject headings across concepts/ — the heading \
         parser is broken and this test is vacuous"
    );

    // 5 or more shared subject headings is a second copy, not a coincidence.
    const LIMIT: usize = 4;

    let mut duplicated = Vec::new();
    for (a, ha) in &concepts {
        for (b, hb) in &pages {
            if b == a || (b.starts_with("concepts/") && b < a) {
                continue;
            }
            let shared: Vec<&String> = ha.intersection(hb).collect();
            if shared.len() > LIMIT {
                duplicated.push(format!(
                    "{a} and {b} share {} headings: {}",
                    shared.len(),
                    shared
                        .iter()
                        .take(8)
                        .map(|s| s.as_str())
                        .collect::<Vec<_>>()
                        .join(", ")
                ));
            }
        }
    }

    assert!(
        duplicated.is_empty(),
        "these pages are explaining the same thing twice:\n  {}\n\n\
         A concepts page holds the language-neutral explanation and links to \
         the per-language reference for the detail. Two hand-maintained copies \
         of the same table drift, and the reader cannot tell which one is \
         current.",
        duplicated.join("\n  ")
    );
}

/// Every link the navigation offers must resolve to a page.
///
/// `every_docs_page_is_reachable_from_the_navigation` checks the other
/// direction — that no page is orphaned — and passed the whole time the site
/// footer, which renders on every documentation page, carried five links to
/// pages that do not exist: `/api`, `/architecture`, `/basic-examples`,
/// `/goals`, and `/complete-beginners-guide`. The last one has no correct
/// target at all; there is no beginner's guide, and its presence in the footer
/// is the likely source of the belief that a separate beginner track exists.
///
/// A route resolves as `<slug>.mdx` or `<slug>/index.mdx` — the same two
/// lookups `lib/mdx.tsx` performs.
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]
fn every_navigation_link_resolves_to_a_page() {
    let Some(docs) = docs_root_or_skip() else {
        return;
    };
    let components = docs.join("components");
    let content = docs.join("content/docs");

    let mut dead: Vec<String> = Vec::new();
    let mut checked = 0usize;

    for file in ["DocsSidebar.tsx", "DocsFooter.tsx", "Breadcrumb.tsx"] {
        let Ok(text) = std::fs::read_to_string(components.join(file)) else {
            continue;
        };
        for raw in text.split("href").skip(1) {
            // `href="/x"` and `href: "/x"` both appear.
            let Some(rest) = raw.split_once('"') else {
                continue;
            };
            let Some((href, _)) = rest.1.split_once('"') else {
                continue;
            };
            // Only internal doc routes; external URLs and anchors are not ours.
            if !href.starts_with('/') || href.starts_with("//") || href.len() < 2 {
                continue;
            }
            let slug = href.trim_start_matches('/');
            checked += 1;
            let direct = content.join(format!("{slug}.mdx"));
            let index = content.join(slug).join("index.mdx");
            if !direct.is_file() && !index.is_file() {
                dead.push(format!("{file}: {href}"));
            }
        }
    }

    assert!(
        checked > 50,
        "parsed too few navigation links ({checked}) — the extractor is broken \
         and this test is vacuous"
    );
    dead.sort();
    dead.dedup();
    assert!(
        dead.is_empty(),
        "navigation links that lead nowhere:\n  {}",
        dead.join("\n  ")
    );
}

/// Prev/Next must not be a second, hand-maintained copy of the sidebar order.
///
/// It was, complete with a comment saying it "must match DocsSidebar.tsx". The
/// two held identical page sets and diverged in position by up to 61 places, so
/// a reader in the Rust section was sent past the Rust Guide into Examples and
/// only reached the guide after all of Python.
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]
fn prev_next_navigation_is_derived_from_the_sidebar() {
    let Some(docs) = docs_root_or_skip() else {
        return;
    };
    let text = std::fs::read_to_string(docs.join("components/PrevNextNav.tsx"))
        .expect("PrevNextNav.tsx must exist");

    assert!(
        text.contains("from \"./DocsSidebar\""),
        "PrevNextNav must import the sidebar's order rather than restate it"
    );
    // A hand-copied list is dozens of literal hrefs; a derived one has none.
    let literal_hrefs = text.matches("href: \"/").count();
    assert!(
        literal_hrefs == 0,
        "PrevNextNav still contains {literal_hrefs} hardcoded page links — that \
         is the copy that drifted"
    );
}
