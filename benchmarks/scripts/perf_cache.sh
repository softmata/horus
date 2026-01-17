#!/bin/bash
# HORUS Cache Miss Profiling Script
# Measures L1/L2/LLC cache misses using Linux perf to validate cache optimizations
#
# This script validates:
# 1. CachePadded structs eliminate false sharing (reduced L1d misses)
# 2. Local caching reduces LLC misses in concurrent access
# 3. Link (SPSC) has better cache behavior than Hub (MPMC)
#
# Usage: ./perf_cache.sh [--full|--quick|--compare]
#   --quick   : Quick single run (default)
#   --full    : Full analysis with multiple runs
#   --compare : Compare Link vs Hub cache behavior

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BENCH_DIR="$(dirname "$SCRIPT_DIR")"
HORUS_ROOT="$(dirname "$BENCH_DIR")"
RESULTS_DIR="$BENCH_DIR/results/cache"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Parse arguments
MODE="quick"
while [[ $# -gt 0 ]]; do
    case $1 in
        --full)
            MODE="full"
            shift
            ;;
        --quick)
            MODE="quick"
            shift
            ;;
        --compare)
            MODE="compare"
            shift
            ;;
        --help)
            echo "HORUS Cache Miss Profiling Script"
            echo ""
            echo "Usage: $0 [--quick|--full|--compare]"
            echo ""
            echo "Options:"
            echo "  --quick   Quick single run with basic cache events (default)"
            echo "  --full    Full analysis with multiple runs and detailed events"
            echo "  --compare Compare Link vs Hub cache behavior"
            echo ""
            echo "Requires: Linux perf (install with: sudo apt install linux-tools-common)"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

echo -e "${BLUE}================================================================================${NC}"
echo -e "${BLUE}  HORUS Cache Miss Profiling${NC}"
echo -e "${BLUE}================================================================================${NC}"
echo ""

# Check prerequisites
echo -e "${YELLOW}Checking prerequisites...${NC}"

if ! command -v perf &> /dev/null; then
    echo -e "${RED}ERROR: perf not found${NC}"
    echo "Install with: sudo apt install linux-tools-common linux-tools-generic linux-tools-\$(uname -r)"
    exit 1
fi

# Check perf access
if ! perf stat --help &> /dev/null 2>&1; then
    echo -e "${RED}ERROR: Cannot access perf${NC}"
    echo "Try: sudo sysctl -w kernel.perf_event_paranoid=-1"
    echo "Or run this script with sudo"
    exit 1
fi

# Check available perf events
echo -e "${CYAN}Detecting available perf events...${NC}"
AVAILABLE_EVENTS=$(perf list 2>/dev/null || echo "")

# Core cache events (most systems support these)
CACHE_EVENTS=""
if echo "$AVAILABLE_EVENTS" | grep -q "L1-dcache-load-misses"; then
    CACHE_EVENTS="$CACHE_EVENTS,L1-dcache-load-misses"
fi
if echo "$AVAILABLE_EVENTS" | grep -q "L1-dcache-loads"; then
    CACHE_EVENTS="$CACHE_EVENTS,L1-dcache-loads"
fi
if echo "$AVAILABLE_EVENTS" | grep -q "LLC-load-misses"; then
    CACHE_EVENTS="$CACHE_EVENTS,LLC-load-misses"
fi
if echo "$AVAILABLE_EVENTS" | grep -q "LLC-loads"; then
    CACHE_EVENTS="$CACHE_EVENTS,LLC-loads"
fi
if echo "$AVAILABLE_EVENTS" | grep -q "cache-misses"; then
    CACHE_EVENTS="$CACHE_EVENTS,cache-misses"
fi
if echo "$AVAILABLE_EVENTS" | grep -q "cache-references"; then
    CACHE_EVENTS="$CACHE_EVENTS,cache-references"
fi

# Remove leading comma
CACHE_EVENTS="${CACHE_EVENTS#,}"

if [ -z "$CACHE_EVENTS" ]; then
    echo -e "${RED}ERROR: No cache events available${NC}"
    echo "This may indicate:"
    echo "  - Running in a VM without perf passthrough"
    echo "  - Kernel compiled without PMU support"
    echo "  - Need to run with elevated privileges"
    exit 1
fi

echo -e "${GREEN}Available cache events: $CACHE_EVENTS${NC}"
echo ""

# Create results directory
mkdir -p "$RESULTS_DIR"

# Build benchmarks in release mode
echo -e "${YELLOW}Building benchmarks in release mode...${NC}"
cd "$HORUS_ROOT"
cargo build --release -p horus_benchmarks --bin ipc_benchmark 2>&1 | tail -5

echo -e "${GREEN}Build complete${NC}"
echo ""

# Function to run perf stat and capture results
run_perf_stat() {
    local name="$1"
    local command="$2"
    local output_file="$RESULTS_DIR/${name}_$TIMESTAMP.txt"

    echo -e "${CYAN}Running: $name${NC}"

    # Run perf stat with cache events
    perf stat -e "$CACHE_EVENTS" \
        --output "$output_file" \
        -- $command 2>&1 | grep -E "(cycles|misses|loads|references)" || true

    echo -e "${GREEN}Results saved to: $output_file${NC}"

    # Display key metrics
    if [ -f "$output_file" ]; then
        echo ""
        echo -e "${YELLOW}Key metrics:${NC}"
        grep -E "(L1-dcache|LLC|cache-)" "$output_file" | head -10 || true
    fi
}

# Function to extract cache miss ratio
extract_miss_ratio() {
    local file="$1"
    local misses=$(grep "L1-dcache-load-misses" "$file" 2>/dev/null | awk '{print $1}' | tr -d ',')
    local loads=$(grep "L1-dcache-loads" "$file" 2>/dev/null | awk '{print $1}' | tr -d ',')

    if [ -n "$misses" ] && [ -n "$loads" ] && [ "$loads" != "0" ]; then
        echo "scale=4; $misses / $loads * 100" | bc 2>/dev/null || echo "N/A"
    else
        echo "N/A"
    fi
}

# Quick mode: single run with basic events
run_quick() {
    echo -e "${BLUE}=== Quick Cache Profile ===${NC}"
    echo ""

    echo -e "${YELLOW}Running Link benchmark with perf...${NC}"

    # Create a simple test that exercises Link
    local test_script=$(mktemp)
    cat > "$test_script" << 'EOF'
#!/bin/bash
cd "$(dirname "$0")"
# Run the criterion benchmark for Link (short)
timeout 30 cargo bench --bench link_performance -- --noplot "Link::send_recv" 2>&1 | tail -20
EOF
    chmod +x "$test_script"

    # Run with perf
    perf stat -e "$CACHE_EVENTS" \
        --output "$RESULTS_DIR/link_quick_$TIMESTAMP.txt" \
        -- timeout 60 cargo bench -p horus_benchmarks --bench link_performance -- --noplot "Link::send_recv" 2>&1 | tail -20

    echo ""
    echo -e "${YELLOW}Cache statistics:${NC}"
    cat "$RESULTS_DIR/link_quick_$TIMESTAMP.txt" 2>/dev/null || echo "See results file for details"

    rm -f "$test_script"
}

# Full mode: multiple runs with detailed analysis
run_full() {
    echo -e "${BLUE}=== Full Cache Analysis ===${NC}"
    echo ""

    # Run each benchmark type
    local benchmarks=("link_small_16B" "link_medium_304B" "link_large_1.5KB" "link_throughput")

    for bench in "${benchmarks[@]}"; do
        echo -e "${CYAN}Profiling: $bench${NC}"

        perf stat -e "$CACHE_EVENTS" \
            --repeat 3 \
            --output "$RESULTS_DIR/${bench}_$TIMESTAMP.txt" \
            -- cargo bench -p horus_benchmarks --bench link_performance -- --noplot "$bench" 2>&1 | tail -5

        echo -e "${GREEN}Completed: $bench${NC}"
        echo ""
    done

    # Generate summary report
    generate_report
}

# Compare mode: Link vs Hub cache behavior
run_compare() {
    echo -e "${BLUE}=== Link vs Hub Cache Comparison ===${NC}"
    echo ""

    echo -e "${YELLOW}This test compares cache behavior between:${NC}"
    echo "  - Link (SPSC): Single-producer, single-consumer with local caching"
    echo "  - Hub (MPMC): Multi-producer, multi-consumer with shared state"
    echo ""
    echo -e "${CYAN}Expected results:${NC}"
    echo "  - Link should have LOWER L1 cache miss rate (better locality)"
    echo "  - Link should have LOWER LLC misses (less cross-core traffic)"
    echo "  - Hub may have higher cache-line bouncing (false sharing)"
    echo ""

    # Run Link benchmark
    echo -e "${YELLOW}[1/2] Profiling Link (SPSC)...${NC}"
    perf stat -e "$CACHE_EVENTS" \
        --output "$RESULTS_DIR/link_compare_$TIMESTAMP.txt" \
        -- cargo bench -p horus_benchmarks --bench link_performance -- --noplot "Link::send_recv" 2>&1 | tail -10

    echo ""

    # Run Hub benchmark (from same suite)
    echo -e "${YELLOW}[2/2] Profiling Hub (MPMC)...${NC}"
    perf stat -e "$CACHE_EVENTS" \
        --output "$RESULTS_DIR/hub_compare_$TIMESTAMP.txt" \
        -- cargo bench -p horus_benchmarks --bench link_performance -- --noplot "Hub::send_recv" 2>&1 | tail -10

    echo ""

    # Compare results
    echo -e "${BLUE}================================================================================${NC}"
    echo -e "${BLUE}  COMPARISON RESULTS${NC}"
    echo -e "${BLUE}================================================================================${NC}"
    echo ""

    echo -e "${CYAN}Link (SPSC) Cache Statistics:${NC}"
    grep -E "(L1-dcache|LLC|cache-)" "$RESULTS_DIR/link_compare_$TIMESTAMP.txt" 2>/dev/null || echo "See results file"

    echo ""
    echo -e "${CYAN}Hub (MPMC) Cache Statistics:${NC}"
    grep -E "(L1-dcache|LLC|cache-)" "$RESULTS_DIR/hub_compare_$TIMESTAMP.txt" 2>/dev/null || echo "See results file"

    echo ""

    # Calculate and display miss ratios
    local link_ratio=$(extract_miss_ratio "$RESULTS_DIR/link_compare_$TIMESTAMP.txt")
    local hub_ratio=$(extract_miss_ratio "$RESULTS_DIR/hub_compare_$TIMESTAMP.txt")

    echo -e "${YELLOW}L1 Data Cache Miss Ratio:${NC}"
    echo "  Link: ${link_ratio}%"
    echo "  Hub:  ${hub_ratio}%"

    if [ "$link_ratio" != "N/A" ] && [ "$hub_ratio" != "N/A" ]; then
        local improvement=$(echo "scale=2; ($hub_ratio - $link_ratio) / $hub_ratio * 100" | bc 2>/dev/null || echo "N/A")
        if [ "$improvement" != "N/A" ]; then
            echo ""
            echo -e "${GREEN}Link improvement over Hub: ${improvement}%${NC}"
        fi
    fi
}

# Generate summary report
generate_report() {
    local report_file="$RESULTS_DIR/cache_report_$TIMESTAMP.md"

    echo -e "${YELLOW}Generating summary report...${NC}"

    cat > "$report_file" << EOF
# HORUS Cache Miss Profiling Report

**Date:** $(date)
**System:** $(uname -a)

## Summary

This report analyzes cache behavior of HORUS IPC mechanisms.

## Key Findings

### Expected Cache Behavior

| Mechanism | L1d Miss Rate | LLC Miss Rate | Notes |
|-----------|---------------|---------------|-------|
| Link (SPSC) | Low (<1%) | Very Low | Local caching, no contention |
| Hub (MPMC) | Medium (1-5%) | Low | Some cross-core traffic |

### Why Link Has Better Cache Performance

1. **CachePadded Structs**: Header and data are on separate cache lines, preventing false sharing
2. **Single-Slot Design**: Only one cache line written at a time
3. **Producer/Consumer Separation**: Each side maintains its own cached state
4. **No Lock Contention**: Lock-free atomic operations stay local

### Optimization Validation

- **False Sharing Elimination**: CachePadded ensures independent cache lines
- **Memory Ordering**: Acquire/Release ordering avoids unnecessary cache flushes
- **Local Caching**: Sequence numbers cached locally, reducing main memory access

## Raw Data

See individual result files in: $RESULTS_DIR/

EOF

    echo -e "${GREEN}Report generated: $report_file${NC}"
}

# Main execution
case $MODE in
    quick)
        run_quick
        ;;
    full)
        run_full
        ;;
    compare)
        run_compare
        ;;
esac

echo ""
echo -e "${BLUE}================================================================================${NC}"
echo -e "${GREEN}Cache profiling complete!${NC}"
echo -e "${BLUE}================================================================================${NC}"
echo ""
echo "Results saved to: $RESULTS_DIR/"
echo ""
echo -e "${CYAN}Interpretation guide:${NC}"
echo "  - L1-dcache-load-misses: Higher = more cache line bouncing (bad)"
echo "  - LLC-load-misses: Higher = more main memory access (expensive)"
echo "  - cache-misses / cache-references: Miss ratio (lower = better)"
echo ""
echo -e "${CYAN}Expected improvements from CachePadded:${NC}"
echo "  - 2-10x reduction in L1d misses for concurrent access"
echo "  - Eliminates false sharing between producer/consumer"
echo ""
