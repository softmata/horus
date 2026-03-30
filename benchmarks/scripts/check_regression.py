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

Absolute Thresholds (nanoseconds) - CI-safe ceilings (~3x target):
    - DirectChannel: 20ns (same-thread, target ~3ns)
    - SpscIntra: 60ns (same-process 1P1C, target ~18ns)
    - SpmcIntra: 80ns (same-process 1PMC, target ~24ns)
    - MpscIntra: 80ns (same-process MP1C, target ~26ns)
    - MpmcIntra: 120ns (same-process MPMC, target ~36ns)
    - PodShm: 150ns (cross-process POD MPMC, target ~50ns)
    - MpscShm: 200ns (cross-process MP1C, target ~65ns)
    - SpmcShm: 200ns (cross-process 1PMC, target ~70ns)
    - SpscShm: 250ns (cross-process 1P1C, target ~85ns)
    - MpmcShm: 600ns (cross-process non-POD MPMC, target ~167ns)
"""

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Absolute latency thresholds (nanoseconds) — CI-safe ceilings (~3x target)
ABSOLUTE_THRESHOLDS = {
    "DirectChannel": 20,
    "DirectChannel_unchecked": 20,
    "SpscIntra": 60,
    "SpmcIntra": 80,
    "MpscIntra": 80,
    "MpmcIntra": 120,
    "PodShm": 150,
    "MpscShm": 200,
    "SpmcShm": 200,
    "SpscShm": 250,
    "MpmcShm": 600,
}

# Backend descriptions for reporting
BACKEND_DESCRIPTIONS = {
    "DirectChannel": "Same-thread pipeline (~3ns)",
    "DirectChannel_unchecked": "Same-thread unchecked (~3ns)",
    "SpscIntra": "Same-process 1P-1C (~18ns)",
    "SpmcIntra": "Same-process 1P-MC (~24ns)",
    "MpscIntra": "Same-process MP-1C (~26ns)",
    "MpmcIntra": "Same-process MPMC (~36ns)",
    "PodShm": "Cross-process POD MPMC (~50ns)",
    "MpscShm": "Cross-process MP-1C (~65ns)",
    "SpmcShm": "Cross-process 1P-MC (~70ns)",
    "SpscShm": "Cross-process 1P-1C (~85ns)",
    "MpmcShm": "Cross-process non-POD MPMC (~167ns)",
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

    # Criterion stores results in: criterion/<group>/<bench>/new/estimates.json
    for group_dir in criterion_dir.iterdir():
        if not group_dir.is_dir() or group_dir.name.startswith("."):
            continue

        for bench_dir in group_dir.iterdir():
            if not bench_dir.is_dir():
                continue

            estimates_path = bench_dir / "new" / "estimates.json"
            if estimates_path.exists():
                try:
                    with open(estimates_path) as f:
                        data = json.load(f)

                    bench_name = bench_dir.name
                    group_name = group_dir.name

                    # Extract median (point estimate)
                    median_ns = data.get("median", {}).get("point_estimate", 0)

                    # Also get mean and stddev if available
                    mean_ns = data.get("mean", {}).get("point_estimate", 0)
                    stddev_ns = data.get("std_dev", {}).get("point_estimate", 0)

                    results[f"{group_name}/{bench_name}"] = {
                        "group": group_name,
                        "name": bench_name,
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
    """
    violations = []

    for full_name, data in results.items():
        bench_name = data["name"]
        median_ns = data["median_ns"]

        # Check if this benchmark has an absolute threshold
        for backend_name, threshold in ABSOLUTE_THRESHOLDS.items():
            if backend_name in bench_name or backend_name in full_name:
                status = "PASS" if median_ns <= threshold else "FAIL"
                violations.append((full_name, median_ns, threshold, status))
                break

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
        lines.append("| Backend | Latency | Threshold | Status |")
        lines.append("|---------|---------|-----------|--------|")

        for full_name, actual, threshold, status in sorted(absolute_checks, key=lambda x: x[0]):
            emoji = "✅" if status == "PASS" else "❌"
            bench_name = full_name.split("/")[-1]
            desc = BACKEND_DESCRIPTIONS.get(bench_name, "")
            lines.append(f"| {bench_name} | {actual:.1f}ns | {threshold}ns | {emoji} {status} |")

        lines.append("")

        # Check for any failures
        failures = [c for c in absolute_checks if c[3] == "FAIL"]
        if failures:
            lines.append("⚠️ **Absolute threshold violations detected!** The following backends exceed their maximum allowed latency:\n")
            for full_name, actual, threshold, _ in failures:
                bench_name = full_name.split("/")[-1]
                lines.append(f"- **{bench_name}**: {actual:.1f}ns (max: {threshold}ns)")
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

    # Summary
    abs_failures = sum(1 for c in absolute_checks if c[3] == "FAIL")
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
    abs_failures = sum(1 for c in absolute_checks if c[3] == "FAIL")
    regressions = sum(1 for c in regression_checks if c[4] == "REGRESSION")

    if abs_failures > 0 or regressions > 0:
        print(f"\n❌ Found {abs_failures} absolute threshold violations and {regressions} regressions")
        sys.exit(1)
    else:
        print("\n✅ All benchmarks pass")
        sys.exit(0)


if __name__ == "__main__":
    main()
