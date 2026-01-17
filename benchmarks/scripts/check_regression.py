#!/usr/bin/env python3
"""
HORUS Performance Regression Checker

Parses Criterion benchmark results and compares against baseline.
Fails CI if any benchmark regresses beyond the threshold.

Usage:
    ./check_regression.py [--threshold 10] [--baseline-file baseline.json] [criterion_dir]

Exit codes:
    0 = No regression
    1 = Regression detected (or error)
"""

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# ANSI colors for terminal output
RED = '\033[0;31m'
GREEN = '\033[0;32m'
YELLOW = '\033[1;33m'
BLUE = '\033[0;34m'
CYAN = '\033[0;36m'
NC = '\033[0m'  # No Color

# Default regression threshold (percentage)
DEFAULT_THRESHOLD = 10.0

# Known benchmark targets for HORUS (in nanoseconds)
# These are the "golden" baselines for HORUS performance
BASELINE_TARGETS = {
    # Link (SPSC) benchmarks - target: <100ns
    "link_small_16B/Link::send_recv": 100.0,
    "link_medium_304B/Link::send_recv": 150.0,
    "link_large_1.5KB/Link::send_recv": 300.0,
    "link_primitives/Link::u32_4B": 80.0,
    "link_primitives/Link::f32_4B": 80.0,
    "link_primitives/Link::f64_8B": 80.0,
    "link_primitives/Link::cmdvel_16B": 100.0,

    # Hub (MPMC) benchmarks - target: ~2-4x Link
    "link_small_16B/Hub::send_recv": 400.0,
    "link_medium_304B/Hub::send_recv": 500.0,
    "link_large_1.5KB/Hub::send_recv": 800.0,
}


def find_estimate_files(criterion_dir: Path) -> Dict[str, Path]:
    """Find all estimates.json files in the criterion output directory."""
    estimates = {}

    for estimates_file in criterion_dir.glob("**/new/estimates.json"):
        # Extract benchmark name from path
        # Path format: target/criterion/<group>/<test>/new/estimates.json
        parts = estimates_file.parts
        try:
            # Find the position of 'criterion' in the path
            criterion_idx = parts.index('criterion')
            # The group and test are the next two components
            group = parts[criterion_idx + 1]
            test = parts[criterion_idx + 2]
            bench_name = f"{group}/{test}"
            estimates[bench_name] = estimates_file
        except (ValueError, IndexError):
            continue

    return estimates


def parse_estimate(path: Path) -> Optional[float]:
    """Parse the mean latency from a Criterion estimates.json file."""
    try:
        with open(path, 'r') as f:
            data = json.load(f)

        # Criterion stores point estimates in nanoseconds
        mean = data.get('mean', {}).get('point_estimate')
        if mean is not None:
            return float(mean)
    except (json.JSONDecodeError, KeyError, TypeError, FileNotFoundError) as e:
        print(f"{YELLOW}Warning: Could not parse {path}: {e}{NC}", file=sys.stderr)

    return None


def load_baseline(baseline_file: Path) -> Dict[str, float]:
    """Load baseline benchmark results from file."""
    if not baseline_file.exists():
        return {}

    try:
        with open(baseline_file, 'r') as f:
            return json.load(f)
    except (json.JSONDecodeError, IOError) as e:
        print(f"{YELLOW}Warning: Could not load baseline {baseline_file}: {e}{NC}", file=sys.stderr)
        return {}


def save_baseline(baseline_file: Path, results: Dict[str, float]):
    """Save current results as new baseline."""
    baseline_file.parent.mkdir(parents=True, exist_ok=True)
    with open(baseline_file, 'w') as f:
        json.dump(results, f, indent=2, sort_keys=True)


def calculate_change(current: float, baseline: float) -> Tuple[float, str]:
    """Calculate percentage change and return formatted string."""
    if baseline == 0:
        return 0.0, "N/A"

    change = ((current - baseline) / baseline) * 100

    if change > 0:
        return change, f"+{change:.1f}%"
    else:
        return change, f"{change:.1f}%"


def check_regressions(
    criterion_dir: Path,
    baseline_file: Path,
    threshold: float,
    save_new_baseline: bool = False,
    use_targets: bool = True
) -> Tuple[bool, List[Dict]]:
    """
    Check for performance regressions.

    Returns:
        (passed, results) where passed is True if no regression exceeds threshold
    """
    estimates = find_estimate_files(criterion_dir)
    baseline = load_baseline(baseline_file)

    if not estimates:
        print(f"{YELLOW}No benchmark results found in {criterion_dir}{NC}")
        return True, []

    current_results = {}
    comparison_results = []
    has_regression = False

    print(f"\n{BLUE}{'='*70}{NC}")
    print(f"{BLUE}  HORUS Performance Regression Check{NC}")
    print(f"{BLUE}{'='*70}{NC}\n")
    print(f"Threshold: {threshold}% regression allowed\n")

    # Sort benchmarks for consistent output
    for bench_name in sorted(estimates.keys()):
        estimates_file = estimates[bench_name]
        current = parse_estimate(estimates_file)

        if current is None:
            continue

        current_results[bench_name] = current

        # Get baseline (prefer saved baseline, fall back to targets)
        if bench_name in baseline:
            baseline_value = baseline[bench_name]
            baseline_source = "saved"
        elif use_targets and bench_name in BASELINE_TARGETS:
            baseline_value = BASELINE_TARGETS[bench_name]
            baseline_source = "target"
        else:
            baseline_value = None
            baseline_source = "none"

        result = {
            "name": bench_name,
            "current": current,
            "baseline": baseline_value,
            "baseline_source": baseline_source,
            "change": None,
            "change_pct": None,
            "status": "ok"
        }

        if baseline_value is not None:
            change_pct, change_str = calculate_change(current, baseline_value)
            result["change"] = change_str
            result["change_pct"] = change_pct

            # Check for regression
            if change_pct > threshold:
                result["status"] = "REGRESSION"
                has_regression = True
                status_color = RED
            elif change_pct > 0:
                result["status"] = "warning"
                status_color = YELLOW
            elif change_pct < -5:
                result["status"] = "improved"
                status_color = GREEN
            else:
                status_color = NC

            print(f"  {bench_name}")
            print(f"    Current:  {current:.1f} ns")
            print(f"    Baseline: {baseline_value:.1f} ns ({baseline_source})")
            print(f"    Change:   {status_color}{change_str}{NC}")

            if result["status"] == "REGRESSION":
                print(f"    {RED}REGRESSION: Exceeds {threshold}% threshold{NC}")

            print()
        else:
            print(f"  {bench_name}")
            print(f"    Current: {current:.1f} ns")
            print(f"    Baseline: N/A (no baseline available)")
            print()

        comparison_results.append(result)

    # Save new baseline if requested
    if save_new_baseline and current_results:
        save_baseline(baseline_file, current_results)
        print(f"{GREEN}Saved new baseline to {baseline_file}{NC}\n")

    return not has_regression, comparison_results


def generate_markdown_report(results: List[Dict], threshold: float) -> str:
    """Generate a markdown report for PR comments."""
    lines = [
        "## Benchmark Results",
        "",
        "| Benchmark | Current (ns) | Baseline (ns) | Change | Status |",
        "|-----------|-------------|---------------|--------|--------|"
    ]

    regressions = []
    improvements = []

    for r in results:
        current = f"{r['current']:.1f}" if r['current'] else "N/A"
        baseline = f"{r['baseline']:.1f}" if r['baseline'] else "N/A"
        change = r['change'] or "N/A"

        status_emoji = {
            "REGRESSION": ":x:",
            "warning": ":warning:",
            "improved": ":rocket:",
            "ok": ":white_check_mark:"
        }.get(r['status'], "")

        lines.append(f"| {r['name']} | {current} | {baseline} | {change} | {status_emoji} |")

        if r['status'] == "REGRESSION":
            regressions.append(r)
        elif r['status'] == "improved":
            improvements.append(r)

    lines.append("")

    if regressions:
        lines.append(f":x: **{len(regressions)} regression(s) detected** (threshold: {threshold}%)")
        lines.append("")
        for r in regressions:
            lines.append(f"- **{r['name']}**: {r['change']} (from {r['baseline']:.1f}ns to {r['current']:.1f}ns)")
    else:
        lines.append(":white_check_mark: **No regressions detected**")

    if improvements:
        lines.append("")
        lines.append(f":rocket: **{len(improvements)} improvement(s)**")
        for r in improvements:
            lines.append(f"- **{r['name']}**: {r['change']}")

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description="Check for performance regressions in Criterion benchmark results"
    )
    parser.add_argument(
        "criterion_dir",
        nargs="?",
        default="target/criterion",
        help="Path to Criterion output directory (default: target/criterion)"
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=DEFAULT_THRESHOLD,
        help=f"Regression threshold in percent (default: {DEFAULT_THRESHOLD})"
    )
    parser.add_argument(
        "--baseline-file",
        type=str,
        default="benchmarks/baseline.json",
        help="Path to baseline file (default: benchmarks/baseline.json)"
    )
    parser.add_argument(
        "--save-baseline",
        action="store_true",
        help="Save current results as new baseline"
    )
    parser.add_argument(
        "--no-targets",
        action="store_true",
        help="Don't use built-in target baselines"
    )
    parser.add_argument(
        "--output-markdown",
        type=str,
        help="Output markdown report to file (for PR comments)"
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Output results as JSON"
    )

    args = parser.parse_args()

    criterion_dir = Path(args.criterion_dir)
    baseline_file = Path(args.baseline_file)

    if not criterion_dir.exists():
        print(f"{RED}Error: Criterion directory not found: {criterion_dir}{NC}")
        sys.exit(1)

    passed, results = check_regressions(
        criterion_dir,
        baseline_file,
        args.threshold,
        save_new_baseline=args.save_baseline,
        use_targets=not args.no_targets
    )

    # Output markdown report
    if args.output_markdown:
        report = generate_markdown_report(results, args.threshold)
        Path(args.output_markdown).write_text(report)
        print(f"Markdown report saved to {args.output_markdown}")

    # Output JSON
    if args.json:
        output = {
            "passed": passed,
            "threshold": args.threshold,
            "results": results
        }
        print(json.dumps(output, indent=2))

    # Summary
    print(f"\n{BLUE}{'='*70}{NC}")
    if passed:
        print(f"{GREEN}PASSED: No performance regressions exceed {args.threshold}% threshold{NC}")
        sys.exit(0)
    else:
        print(f"{RED}FAILED: Performance regression detected (>{args.threshold}% threshold){NC}")
        sys.exit(1)


if __name__ == "__main__":
    main()
