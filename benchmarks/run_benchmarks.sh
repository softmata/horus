#!/bin/bash
# HORUS Benchmark Runner
#
# Runs the complete benchmark suite with proper isolation and outputs
# JSON results for CI regression tracking.
#
# Usage:
#   ./run_benchmarks.sh              # Run all benchmarks
#   ./run_benchmarks.sh --quick      # Quick validation run
#   ./run_benchmarks.sh --criterion  # Criterion benches only
#   ./run_benchmarks.sh --perf       # Performance binaries only

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Output directory
OUTPUT_DIR="${SCRIPT_DIR}/results"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
RESULTS_DIR="${OUTPUT_DIR}/${TIMESTAMP}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_header() {
    echo -e "${BLUE}"
    echo "╔══════════════════════════════════════════════════════════════════╗"
    echo "║           HORUS Benchmark Suite                                  ║"
    echo "╠══════════════════════════════════════════════════════════════════╣"
    echo "║  Industry-grade benchmarks for robotics IPC                      ║"
    echo "╚══════════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_section() {
    echo -e "\n${GREEN}▶ $1${NC}\n"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

# Parse arguments
QUICK=false
CRITERION_ONLY=false
PERF_ONLY=false

for arg in "$@"; do
    case $arg in
        --quick)
            QUICK=true
            ;;
        --criterion)
            CRITERION_ONLY=true
            ;;
        --perf)
            PERF_ONLY=true
            ;;
        --help|-h)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --quick      Quick validation run with reduced iterations"
            echo "  --criterion  Run only Criterion benchmarks"
            echo "  --perf       Run only performance binary benchmarks"
            echo "  --help       Show this help message"
            exit 0
            ;;
    esac
done

print_header

# Create output directory
mkdir -p "$RESULTS_DIR"
echo "Results will be saved to: $RESULTS_DIR"
echo ""

# Check CPU governor
print_section "System Preparation"

CPU_GOVERNOR=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo "unknown")
if [ "$CPU_GOVERNOR" != "performance" ]; then
    print_warning "CPU governor is '$CPU_GOVERNOR', not 'performance'"
    print_warning "For accurate results, run: sudo cpupower frequency-set -g performance"
fi

# Check for other processes using CPU
HIGH_CPU_PROCS=$(ps aux --sort=-%cpu | head -5 | tail -4 | awk '$3 > 10 {print $11}')
if [ -n "$HIGH_CPU_PROCS" ]; then
    print_warning "High CPU usage detected from other processes"
fi

# Build benchmarks
print_section "Building Benchmarks"

if $QUICK; then
    echo "Building in release mode (quick run)..."
else
    echo "Building in release mode with optimizations..."
fi

cargo build --release --workspace 2>&1 | tail -5

print_success "Build complete"

# Run Criterion benchmarks
if ! $PERF_ONLY; then
    print_section "Running Criterion Benchmarks"

    CRITERION_BENCHES=(
        "latency_rigorous"
        "competitive_comparison"
        "latency_matrix"
        "topic_performance"
    )

    for bench in "${CRITERION_BENCHES[@]}"; do
        echo "Running: $bench"
        if $QUICK; then
            cargo bench --bench "$bench" -- --sample-size 10 --measurement-time 1 2>&1 | \
                grep -E "(time:|thrpt:|Benchmarking|latency)" | head -20
        else
            cargo bench --bench "$bench" 2>&1 | \
                grep -E "(time:|thrpt:|Benchmarking|latency)" | head -50
        fi
        echo ""
    done

    # Copy Criterion HTML reports
    if [ -d "target/criterion" ]; then
        cp -r target/criterion "$RESULTS_DIR/criterion_reports"
        print_success "Criterion reports saved to: $RESULTS_DIR/criterion_reports"
    fi
fi

# Run performance binary benchmarks
if ! $CRITERION_ONLY; then
    print_section "Running Performance Benchmarks"

    # Determinism benchmark
    echo "Running: determinism_benchmark"
    if $QUICK; then
        cargo run --release --bin determinism_benchmark -- \
            --iterations 10000 \
            --runs 3 \
            --json "$RESULTS_DIR/determinism.json" 2>&1 | tail -30
    else
        cargo run --release --bin determinism_benchmark -- \
            --json "$RESULTS_DIR/determinism.json" 2>&1 | tail -50
    fi
    echo ""

    # Scalability benchmark
    echo "Running: scalability_benchmark"
    cargo run --release --bin scalability_benchmark -- \
        --json "$RESULTS_DIR/scalability.json" 2>&1 | tail -40
    echo ""

    # IPC benchmark
    echo "Running: ipc_benchmark"
    cargo run --release --bin ipc_benchmark 2>&1 | tail -30
    echo ""

    # POD benchmark
    echo "Running: pod_benchmark"
    cargo run --release --bin pod_benchmark 2>&1 | tail -30
    echo ""

    # Cross-process benchmark (TRUE IPC measurement)
    echo "Running: cross_process_benchmark"
    cargo run --release --bin cross_process_benchmark -- \
        --json "$RESULTS_DIR/cross_process.json" 2>&1 | tail -40
    echo ""

    # Robotics messages benchmark
    echo "Running: robotics_messages_benchmark"
    cargo run --release --bin robotics_messages_benchmark -- \
        --json "$RESULTS_DIR/robotics_messages.json" 2>&1 | tail -50
    echo ""

    # DDS comparison benchmark
    echo "Running: dds_comparison_benchmark"
    cargo run --release --bin dds_comparison_benchmark -- \
        --json "$RESULTS_DIR/dds_comparison.json" 2>&1 | tail -50
    echo ""
fi

# Generate summary
print_section "Generating Summary"

SUMMARY_FILE="$RESULTS_DIR/summary.txt"
{
    echo "HORUS Benchmark Results"
    echo "======================="
    echo ""
    echo "Timestamp: $(date)"
    echo "Git Commit: $(git rev-parse HEAD 2>/dev/null || echo 'unknown')"
    echo "Git Branch: $(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo 'unknown')"
    echo ""
    echo "Platform:"
    echo "  CPU: $(cat /proc/cpuinfo | grep 'model name' | head -1 | cut -d: -f2 | xargs)"
    echo "  Cores: $(nproc)"
    echo "  Memory: $(free -h | grep Mem | awk '{print $2}')"
    echo "  Governor: $CPU_GOVERNOR"
    echo ""
    echo "Results Directory: $RESULTS_DIR"
    echo ""
    echo "Files:"
    ls -la "$RESULTS_DIR"
} > "$SUMMARY_FILE"

cat "$SUMMARY_FILE"

print_section "Complete"
print_success "All benchmarks completed successfully"
echo ""
echo "Results saved to: $RESULTS_DIR"
echo "View Criterion reports: open $RESULTS_DIR/criterion_reports/report/index.html"
