//! A performance claim on the front page has to be traceable to something.
//!
//! The README led with "575x faster than ROS2" and carried a table of
//! comparisons:
//!
//! ```text
//! | Same-process pub/sub | 91 ns  | ~50 µs  (550x) |
//! | Cross-process        | 171 ns | ~100 µs (585x) |
//! | 1 pub → 3 subs       | 80 ns  | ~70 µs  (875x) |
//! ```
//!
//! Three things were wrong with it.
//!
//! **No ROS 2 was measured.** `dds_comparison_benchmark` builds without the
//! `dds` feature by default, and on that path it fabricated `BenchmarkResult`
//! entries for ROS2, CycloneDDS, FastDDS and iceoryx — each with a full
//! percentile distribution (p1 through p99.99 plus confidence bounds) computed
//! arithmetically from two hardcoded constants — and wrote them into the JSON
//! report next to real measurements, with nothing in the schema distinguishing
//! them. `count: 0` and an `_reference` suffix were the only hints.
//!
//! **The numbers contradicted the project's own source.** The README used
//! ~50 µs for ROS 2 same-process. The only ROS 2 figure anywhere in the
//! repository is `dds_comparison_benchmark`'s ~5,000 ns, cited to REP 2014 —
//! ten times lower. The 50 µs figure appears in no benchmark, no report, and no
//! source.
//!
//! **The comparison was not like-for-like.** 91 ns and 80 ns are producer-side
//! `send()` latencies (`all_paths_latency` labels them `[send]`); a DDS
//! comparison figure is end-to-end. Only the 171 ns cross-process row is
//! one-way.
//!
//! The docs went further and called it "550-875x faster than ROS2 (DDS) in
//! measured pub/sub benchmarks" — asserting a measurement that did not exist.
//!
//! These tests do not check whether any particular number is *right*. They
//! check that the claims stay traceable: that a competitor figure carries its
//! provenance, and that the front page does not reintroduce a ratio the
//! repository cannot support.
//!
//! Run: `cargo test -p horus_manager --test perf_claims_contract`

use std::path::{Path, PathBuf};

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager must have a parent")
        .to_path_buf()
}

fn read(rel: &str) -> Option<String> {
    std::fs::read_to_string(repo_root().join(rel)).ok()
}

/// The benchmark result schema must be able to say where a number came from.
#[test]
fn benchmark_results_record_their_provenance() {
    let lib = read("benchmarks/src/lib.rs").expect("benchmarks/src/lib.rs must exist");

    assert!(
        lib.contains("pub enum Provenance"),
        "BenchmarkResult has no way to distinguish a measured number from one \
         quoted out of a paper, so a report can present both as equivalent"
    );
    assert!(
        lib.contains("Literature"),
        "Provenance must have a variant for figures taken from published work"
    );
    assert!(
        lib.contains("pub provenance: Provenance"),
        "BenchmarkResult must carry the provenance, not merely define the type"
    );
}

/// Quoted competitor figures must be tagged, or they read as measurements.
#[test]
fn quoted_competitor_figures_are_tagged_as_literature() {
    let bench =
        read("benchmarks/src/bin/dds_comparison_benchmark.rs").expect("dds benchmark must exist");

    assert!(
        bench.contains("Provenance::Literature"),
        "the synthetic ROS2/DDS reference results are still emitted as if \
         measured"
    );
    assert!(
        bench.contains("REP2014_SOURCE") || bench.contains("REP 2014"),
        "a quoted figure must name its source specifically enough to look up"
    );
}

/// The inflated ratios must not come back without evidence appearing with them.
#[test]
fn the_readme_does_not_claim_an_unsupported_ratio() {
    let readme = read("README.md").expect("README.md must exist");

    for claim in ["575x", "550x", "585x", "875x", "550-875x"] {
        assert!(
            !readme.contains(claim),
            "README.md claims `{claim}` against ROS 2. The only ROS 2 figure in \
             this repository is REP 2014's ~5,000 ns, which puts the ratio near \
             30x against HORUS's end-to-end 171 ns. If a larger ratio is real, \
             add the benchmark that produces it and cite it here — the number \
             alone is not evidence."
        );
    }
}

/// A comparison table has to say what was measured. Comparing send-side latency
/// to a competitor's end-to-end latency is the error that produced the original
/// figures.
#[test]
fn the_readme_states_what_its_latencies_measure() {
    let readme = read("README.md").expect("README.md must exist");

    assert!(
        readme.contains("producer-side") && readme.contains("one-way"),
        "the latency table should say which rows are producer-side `send()` and \
         which are end-to-end, since they are not comparable to each other, let \
         alone to another framework's numbers"
    );
    assert!(
        readme.contains("all_paths_latency"),
        "the table should name the benchmark that reproduces it"
    );
}

/// Claiming a competitor was measured when it was not is the sharpest version
/// of this defect.
#[test]
fn the_docs_do_not_claim_an_unmeasured_comparison_was_measured() {
    let Some(root) = repo_root()
        .parent()
        .map(|p| p.join("horus-docs/content/docs"))
        .filter(|p| p.is_dir())
    else {
        eprintln!("SKIP: horus-docs is not checked out beside horus");
        return;
    };

    let mut offenders = Vec::new();
    let mut stack = vec![root.clone()];
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
            if !path.extension().is_some_and(|e| e == "mdx" || e == "md") {
                continue;
            }
            let Ok(text) = std::fs::read_to_string(&path) else {
                continue;
            };
            for (i, line) in text.lines().enumerate() {
                let lower = line.to_lowercase();
                let claims_ros2 = lower.contains("ros2") || lower.contains("ros 2");
                let claims_measured = lower.contains("measured") || lower.contains("benchmark");
                let has_big_ratio = ["550", "575", "585", "875"]
                    .iter()
                    .any(|r| line.contains(r));
                if claims_ros2 && claims_measured && has_big_ratio {
                    let rel = path.strip_prefix(&root).unwrap_or(&path);
                    offenders.push(format!("{}:{}: {}", rel.display(), i + 1, line.trim()));
                }
            }
        }
    }

    assert!(
        offenders.is_empty(),
        "these lines present a ROS 2 comparison as measured, but nothing in the \
         repository measures ROS 2 unless built with `-F dds` and a DDS \
         implementation installed:\n  {}",
        offenders.join("\n  ")
    );
}
