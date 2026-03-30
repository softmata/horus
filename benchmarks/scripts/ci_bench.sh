#!/bin/bash
# HORUS Benchmark CI Integration
#
# Runs all criterion benchmarks, compares against baseline, and fails
# if any benchmark regresses by more than the threshold.
#
# Usage:
#   ./ci_bench.sh                    # Run benchmarks, compare to baseline
#   ./ci_bench.sh --save-baseline    # Run benchmarks and save as new baseline
#   ./ci_bench.sh --quick            # Run only critical path benchmarks
#
# Environment variables:
#   BENCH_THRESHOLD     Regression threshold percentage (default: 10)
#   BENCH_BASELINE      Path to baseline JSON file (default: benchmarks/baseline.json)
#   BENCH_REPORT        Path to markdown report output (default: benchmark_report.md)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$SCRIPT_DIR/../.."
BENCH_DIR="$SCRIPT_DIR/.."

# Defaults
THRESHOLD="${BENCH_THRESHOLD:-10}"
BASELINE="${BENCH_BASELINE:-$BENCH_DIR/baseline.json}"
REPORT="${BENCH_REPORT:-$REPO_ROOT/benchmark_report.md}"
CRITERION_DIR="$REPO_ROOT/target/criterion"
SAVE_BASELINE=false
QUICK=false

# Parse args
for arg in "$@"; do
    case $arg in
        --save-baseline) SAVE_BASELINE=true ;;
        --quick) QUICK=true ;;
        *) echo "Unknown argument: $arg"; exit 1 ;;
    esac
done

echo "============================================"
echo "  HORUS Benchmark CI"
echo "============================================"
echo "  Threshold: ${THRESHOLD}%"
echo "  Baseline:  $BASELINE"
echo "  Report:    $REPORT"
echo "============================================"
echo ""

# Clean SHM before benchmarks
echo "Cleaning shared memory..."
rm -rf /dev/shm/horus_*/tensors/ 2>/dev/null || true
rm -rf /dev/shm/horus_*/topics/ 2>/dev/null || true
rm -rf /dev/shm/horus_*/nodes/ 2>/dev/null || true

# Build benchmarks in release mode
echo "Building benchmarks..."
cd "$REPO_ROOT"
cargo bench --no-run -p horus_benchmarks 2>&1 | tail -5

# Run criterion benchmarks
if [ "$QUICK" = true ]; then
    echo "Running critical path benchmarks (quick mode)..."
    cargo bench -p horus_benchmarks -- "topic_latency|tensor_alloc_release" 2>&1 | tail -20
else
    echo "Running all criterion benchmarks..."
    cargo bench -p horus_benchmarks 2>&1 | tail -40
fi

echo ""
echo "Benchmarks complete. Analyzing results..."

# Check if results exist
if [ ! -d "$CRITERION_DIR" ]; then
    echo "ERROR: Criterion results not found at $CRITERION_DIR"
    exit 1
fi

# Run regression checker
CHECKER_ARGS="$CRITERION_DIR --threshold $THRESHOLD --output-markdown $REPORT"

if [ "$SAVE_BASELINE" = true ]; then
    CHECKER_ARGS="$CHECKER_ARGS --baseline-file $BASELINE --save-baseline"
    echo "Saving baseline to $BASELINE..."
elif [ -f "$BASELINE" ]; then
    CHECKER_ARGS="$CHECKER_ARGS --baseline-file $BASELINE"
    echo "Comparing against baseline..."
else
    echo "No baseline found at $BASELINE — running absolute threshold checks only."
    echo "Run with --save-baseline to create one."
fi

python3 "$SCRIPT_DIR/check_regression.py" $CHECKER_ARGS
EXIT_CODE=$?

# Clean SHM after benchmarks
rm -rf /dev/shm/horus_*/tensors/ 2>/dev/null || true

if [ $EXIT_CODE -eq 0 ]; then
    echo ""
    echo "============================================"
    echo "  All benchmarks PASS"
    echo "============================================"
else
    echo ""
    echo "============================================"
    echo "  BENCHMARK REGRESSION DETECTED"
    echo "  See $REPORT for details"
    echo "============================================"
fi

exit $EXIT_CODE
