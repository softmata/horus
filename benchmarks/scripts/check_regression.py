#!/usr/bin/env python3
"""
HORUS Benchmark Regression Checker

Analyzes Criterion benchmark results and checks for performance regressions.
Supports both percentage-based and absolute latency thresholds.

Usage:
    python check_regression.py <criterion_dir> [options]

Options:
    --threshold <percent>     Fail if any benchmark regresses by more than this percent (default: 10)
    --baseline-file <path>    Path to baseline JSON file
    --save-baseline           Save current results as new baseline
    --output-markdown <path>  Write markdown summary to file
    --absolute-thresholds     Enable absolute latency thresholds for critical paths

Absolute thresholds are keyed on the exact Criterion benchmark id — the path
Criterion writes under target/criterion, e.g. "native_shm_latency/small/16B".
They used to be keyed on backend names (DirectChannel, SpscIntra, PodShm, ...)
and matched by substring. No Criterion bench emits those names — six of them
name backends that no longer exist at all — so the substring test was never
true, zero checks ran, and the gate reported "All benchmarks pass" for every
change, including one that tripled Topic::send latency. Two consequences of
that are guarded against below: an id in the table that no run produces is an
error, and a run in which zero checks fired is an error.

The ceilings themselves are deliberately loose. There is no published
baseline.json in this tree, so they are provisional: wide enough not to fire on
a shared CI runner's noise (a suite that fails for reasons outside the
repository teaches people to ignore it), tight enough to catch an order-of-
magnitude regression. Tighten them once --save-baseline has produced a real
baseline; the percentage check against that baseline, not these ceilings, is
the gate that should catch a 10% regression.
"""

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Absolute latency ceilings (nanoseconds), keyed on the exact Criterion
# benchmark id. Keys are matched exactly, not by substring: a renamed bench must
# drop out of the table loudly (see check_absolute_thresholds) rather than
# silently stop being checked.
ABSOLUTE_THRESHOLDS = {
    "native_shm_latency/small/16B": 500,
    "native_shm_latency/medium/128B": 700,
    "native_shm_latency/large/4KB": 3000,
    "latency_comparison/native_shm_16B": 500,
    "pubsub_roundtrip/native_roundtrip_16B": 500,
    "pubsub_roundtrip/native_roundtrip_1KB": 2000,
}

# What each checked benchmark actually times, for the report.
BENCH_DESCRIPTIONS = {
    "native_shm_latency/small/16B": "SHM topic send+recv, 16B POD",
    "native_shm_latency/medium/128B": "SHM topic send+recv, 128B POD",
    "native_shm_latency/large/4KB": "SHM topic send+recv, 4KB POD",
    "latency_comparison/native_shm_16B": "SHM topic send+recv, 16B POD",
    "pubsub_roundtrip/native_roundtrip_16B": "Pub-sub round trip, 16B",
    "pubsub_roundtrip/native_roundtrip_1KB": "Pub-sub round trip, 1KB",
}


def find_benchmark_results(criterion_dir: Path) -> Dict[str, Dict]:
    """
    Find and parse all benchmark results from Criterion output directory.
    Returns dict mapping benchmark name to results.
    """
    results = {}

    if not criterion_dir.exists():
        print(f"Warning: Criterion directory not found: {criterion_dir}")
        return results

    # Criterion nests one directory per BenchmarkId component, so a bench
    # declared as BenchmarkId::new("small", "16B") inside group
    # "native_shm_latency" lands at native_shm_latency/small/16B/new/. The old
    # two-level `for group_dir / for bench_dir` walk only ever saw the flat
    # `group/bench` case, so every parameterised benchmark — which is most of
    # them — was invisible to the checker. Recurse instead, and key each result
    # on its full Criterion path.
    for estimates_path in sorted(criterion_dir.rglob("new/estimates.json")):
        bench_dir = estimates_path.parent.parent
        parts = bench_dir.relative_to(criterion_dir).parts

        # "report" holds Criterion's generated HTML, not measurements.
        if not parts or parts[0] == "report" or any(p.startswith(".") for p in parts):
            continue

        full_name = "/".join(parts)

        try:
            with open(estimates_path) as f:
                data = json.load(f)

            # Extract median (point estimate)
            median_ns = data.get("median", {}).get("point_estimate", 0)

            # Also get mean and stddev if available
            mean_ns = data.get("mean", {}).get("point_estimate", 0)
            stddev_ns = data.get("std_dev", {}).get("point_estimate", 0)

            results[full_name] = {
                "group": parts[0],
                "name": bench_dir.name,
                "median_ns": median_ns,
                "mean_ns": mean_ns,
                "stddev_ns": stddev_ns,
            }

        except (json.JSONDecodeError, KeyError) as e:
            print(f"Warning: Failed to parse {estimates_path}: {e}")

    return results


def check_absolute_thresholds(results: Dict[str, Dict]) -> List[Tuple[str, float, float, str]]:
    """
    Check results against absolute latency thresholds.
    Returns list of (benchmark, actual_ns, threshold_ns, status) tuples.

    Matching is exact on the full Criterion id. It used to be a substring test
    against backend names no bench emits, which matched nothing and made the
    gate unconditionally green. A threshold entry with no matching result is
    reported as MISSING and counted as a failure, so renaming a bench cannot
    quietly drop it out of the gate.
    """
    violations = []

    for full_name, data in results.items():
        threshold = ABSOLUTE_THRESHOLDS.get(full_name)
        if threshold is None:
            continue

        median_ns = data["median_ns"]
        status = "PASS" if median_ns <= threshold else "FAIL"
        violations.append((full_name, median_ns, threshold, status))

    for name, threshold in ABSOLUTE_THRESHOLDS.items():
        if name not in results:
            violations.append((name, float("nan"), threshold, "MISSING"))

    return violations


def check_percentage_regression(
    results: Dict[str, Dict],
    baseline: Dict[str, Dict],
    threshold_percent: float,
) -> List[Tuple[str, float, float, float, str]]:
    """
    Check results against baseline for percentage-based regression.
    Returns list of (benchmark, current_ns, baseline_ns, change_percent, status) tuples.
    """
    regressions = []

    for full_name, data in results.items():
        if full_name not in baseline:
            continue

        current_ns = data["median_ns"]
        baseline_ns = baseline[full_name]["median_ns"]

        if baseline_ns == 0:
            continue

        change_percent = ((current_ns - baseline_ns) / baseline_ns) * 100

        if change_percent > threshold_percent:
            status = "REGRESSION"
        elif change_percent < -threshold_percent:
            status = "IMPROVEMENT"
        else:
            status = "UNCHANGED"

        regressions.append((full_name, current_ns, baseline_ns, change_percent, status))

    return regressions


def generate_markdown_report(
    results: Dict[str, Dict],
    absolute_checks: List[Tuple[str, float, float, str]],
    regression_checks: List[Tuple[str, float, float, float, str]],
    threshold_percent: float,
) -> str:
    """Generate a Markdown summary of benchmark results."""
    lines = ["## 🏃 Benchmark Results\n"]

    # Absolute threshold section
    if absolute_checks:
        lines.append("### Critical Path Latencies\n")
        lines.append("| Benchmark | What it times | Latency | Threshold | Status |")
        lines.append("|-----------|---------------|---------|-----------|--------|")

        for full_name, actual, threshold, status in sorted(absolute_checks, key=lambda x: x[0]):
            emoji = "✅" if status == "PASS" else "❌"
            desc = BENCH_DESCRIPTIONS.get(full_name, "")
            actual_str = "—" if status == "MISSING" else f"{actual:.1f}ns"
            lines.append(
                f"| `{full_name}` | {desc} | {actual_str} | {threshold}ns | {emoji} {status} |"
            )

        lines.append("")

        # Check for any failures
        failures = [c for c in absolute_checks if c[3] == "FAIL"]
        if failures:
            lines.append("⚠️ **Absolute threshold violations detected!** The following benchmarks exceed their maximum allowed latency:\n")
            for full_name, actual, threshold, _ in failures:
                lines.append(f"- **`{full_name}`**: {actual:.1f}ns (max: {threshold}ns)")
            lines.append("")

        # A benchmark that the threshold table names but the run never produced
        # is a hole in the gate, not a pass: the bench was renamed or removed
        # and nothing is checking that path any more.
        missing = [c for c in absolute_checks if c[3] == "MISSING"]
        if missing:
            lines.append("⚠️ **Benchmarks in the threshold table produced no results.** They were renamed or removed, and are no longer gated:\n")
            for full_name, _actual, _threshold, _ in missing:
                lines.append(f"- **`{full_name}`**")
            lines.append("")

    # Percentage regression section
    if regression_checks:
        lines.append("### Regression Analysis\n")
        lines.append(f"*Threshold: ±{threshold_percent}%*\n")
        lines.append("| Benchmark | Current | Baseline | Change | Status |")
        lines.append("|-----------|---------|----------|--------|--------|")

        for full_name, current, baseline, change, status in sorted(regression_checks, key=lambda x: -abs(x[3])):
            if status == "REGRESSION":
                emoji = "🔴"
            elif status == "IMPROVEMENT":
                emoji = "🟢"
            else:
                emoji = "⚪"

            bench_name = full_name.split("/")[-1]
            change_str = f"+{change:.1f}%" if change > 0 else f"{change:.1f}%"
            lines.append(f"| {bench_name} | {current:.1f}ns | {baseline:.1f}ns | {change_str} | {emoji} |")

        lines.append("")

    # Summary. MISSING counts as a failure: an ungated critical path is not a
    # pass, and treating it as one is how this gate stayed green for every
    # change ever made to it.
    abs_failures = sum(1 for c in absolute_checks if c[3] in ("FAIL", "MISSING"))
    regressions = sum(1 for c in regression_checks if c[4] == "REGRESSION")

    if abs_failures > 0 or regressions > 0:
        lines.append(f"### Summary: ❌ {abs_failures + regressions} issue(s) found\n")
    else:
        lines.append("### Summary: ✅ All benchmarks pass\n")

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="HORUS Benchmark Regression Checker")
    parser.add_argument("criterion_dir", help="Path to Criterion results directory")
    parser.add_argument("--threshold", type=float, default=10, help="Percentage regression threshold")
    parser.add_argument("--baseline-file", help="Path to baseline JSON file")
    parser.add_argument("--save-baseline", action="store_true", help="Save current results as baseline")
    parser.add_argument("--output-markdown", help="Write markdown summary to file")
    parser.add_argument("--absolute-thresholds", action="store_true", default=True,
                        help="Enable absolute latency threshold checks (default: true)")
    parser.add_argument("--no-absolute-thresholds", action="store_false", dest="absolute_thresholds",
                        help="Disable absolute latency threshold checks")

    args = parser.parse_args()

    criterion_dir = Path(args.criterion_dir)
    results = find_benchmark_results(criterion_dir)

    if not results:
        print("No benchmark results found!")
        sys.exit(1)

    print(f"Found {len(results)} benchmark results")

    # Load baseline if available
    baseline = {}
    if args.baseline_file and os.path.exists(args.baseline_file):
        with open(args.baseline_file) as f:
            baseline = json.load(f)
        print(f"Loaded baseline with {len(baseline)} entries")

    # Save baseline if requested
    if args.save_baseline and args.baseline_file:
        with open(args.baseline_file, "w") as f:
            json.dump(results, f, indent=2)
        print(f"Saved baseline to {args.baseline_file}")

    # Run checks
    absolute_checks = []
    regression_checks = []

    if args.absolute_thresholds:
        absolute_checks = check_absolute_thresholds(results)
        print(f"Ran {len(absolute_checks)} absolute threshold checks")

        # The whole point of the gate. Absolute checks are on by default, so a
        # run that fired none of them means the threshold table no longer
        # matches any benchmark id — which is exactly the state this file was
        # in, silently, while printing "All benchmarks pass".
        if not absolute_checks:
            print(
                "ERROR: 0 absolute threshold checks ran — the threshold table "
                "matches no benchmark id.\n  Known ids: " + ", ".join(sorted(results))
            )
            sys.exit(1)

    if baseline:
        regression_checks = check_percentage_regression(results, baseline, args.threshold)
        print(f"Ran {len(regression_checks)} regression checks")

    # Generate report
    report = generate_markdown_report(results, absolute_checks, regression_checks, args.threshold)

    if args.output_markdown:
        with open(args.output_markdown, "w") as f:
            f.write(report)
        print(f"Wrote report to {args.output_markdown}")

    print("\n" + report)

    # Determine exit code
    abs_failures = sum(1 for c in absolute_checks if c[3] in ("FAIL", "MISSING"))
    regressions = sum(1 for c in regression_checks if c[4] == "REGRESSION")

    if abs_failures > 0 or regressions > 0:
        print(f"\n❌ Found {abs_failures} absolute threshold violations and {regressions} regressions")
        sys.exit(1)
    else:
        print("\n✅ All benchmarks pass")
        sys.exit(0)


if __name__ == "__main__":
    main()
