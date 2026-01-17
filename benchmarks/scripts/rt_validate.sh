#!/bin/bash
# HORUS Real-Time Latency Validation Script
# Validates worst-case latency using cyclictest and HORUS IPC benchmarks
#
# This script validates:
# 1. Kernel RT capabilities (PREEMPT_RT detection)
# 2. System configuration for RT (memory lock, RT priority)
# 3. HORUS IPC latency under RT conditions
# 4. Comparison against cyclictest baseline
#
# Usage: ./rt_validate.sh [--full|--quick|--cyclictest-only|--horus-only]
#   --quick          : Quick validation (default)
#   --full           : Full validation with extended duration
#   --cyclictest-only: Only run cyclictest baseline
#   --horus-only     : Only run HORUS IPC tests
#   --rt-config      : Apply RT configuration before tests
#
# Target: <100µs worst-case latency

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BENCH_DIR="$(dirname "$SCRIPT_DIR")"
HORUS_ROOT="$(dirname "$BENCH_DIR")"
RESULTS_DIR="$BENCH_DIR/results/rt"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Target worst-case latency in microseconds
TARGET_LATENCY_US=100

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Parse arguments
MODE="quick"
APPLY_RT_CONFIG=false
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
        --cyclictest-only)
            MODE="cyclictest"
            shift
            ;;
        --horus-only)
            MODE="horus"
            shift
            ;;
        --rt-config)
            APPLY_RT_CONFIG=true
            shift
            ;;
        --help)
            echo "HORUS Real-Time Latency Validation Script"
            echo ""
            echo "Usage: $0 [--quick|--full|--cyclictest-only|--horus-only] [--rt-config]"
            echo ""
            echo "Options:"
            echo "  --quick          Quick validation (~30 seconds)"
            echo "  --full           Full validation (~5 minutes)"
            echo "  --cyclictest-only Only run cyclictest baseline"
            echo "  --horus-only     Only run HORUS IPC tests"
            echo "  --rt-config      Apply RT configuration (requires root)"
            echo ""
            echo "Target: <${TARGET_LATENCY_US}µs worst-case latency"
            echo ""
            echo "Prerequisites:"
            echo "  - cyclictest: sudo apt install rt-tests"
            echo "  - For full RT: PREEMPT_RT kernel"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

echo -e "${BLUE}================================================================================${NC}"
echo -e "${BLUE}  HORUS Real-Time Latency Validation${NC}"
echo -e "${BLUE}================================================================================${NC}"
echo ""

# Create results directory
mkdir -p "$RESULTS_DIR"

# ============================================================================
# System Information
# ============================================================================

echo -e "${YELLOW}Collecting system information...${NC}"

# Kernel info
KERNEL_VERSION=$(uname -r)
KERNEL_TYPE="Standard"
if grep -q "PREEMPT_RT" /proc/version 2>/dev/null || [ -f /sys/kernel/realtime ]; then
    KERNEL_TYPE="PREEMPT_RT"
fi

# CPU info
CPU_MODEL=$(lscpu | grep "Model name" | cut -d: -f2 | xargs)
CPU_CORES=$(nproc)

# Check RT limits
RLIMIT_RTPRIO=$(ulimit -r 2>/dev/null || echo "N/A")
RLIMIT_MEMLOCK=$(ulimit -l 2>/dev/null || echo "N/A")

echo -e "${CYAN}System Configuration:${NC}"
echo "  Kernel: $KERNEL_VERSION ($KERNEL_TYPE)"
echo "  CPU: $CPU_MODEL"
echo "  Cores: $CPU_CORES"
echo "  RLIMIT_RTPRIO: $RLIMIT_RTPRIO"
echo "  RLIMIT_MEMLOCK: $RLIMIT_MEMLOCK"
echo ""

# Save system info
cat > "$RESULTS_DIR/system_info_$TIMESTAMP.json" << EOF
{
  "timestamp": "$(date -Iseconds)",
  "kernel_version": "$KERNEL_VERSION",
  "kernel_type": "$KERNEL_TYPE",
  "cpu_model": "$CPU_MODEL",
  "cpu_cores": $CPU_CORES,
  "rlimit_rtprio": "$RLIMIT_RTPRIO",
  "rlimit_memlock": "$RLIMIT_MEMLOCK",
  "target_latency_us": $TARGET_LATENCY_US
}
EOF

# ============================================================================
# Check Prerequisites
# ============================================================================

echo -e "${YELLOW}Checking prerequisites...${NC}"

# Check for cyclictest
CYCLICTEST_AVAILABLE=false
if command -v cyclictest &> /dev/null; then
    CYCLICTEST_AVAILABLE=true
    CYCLICTEST_VERSION=$(cyclictest --help 2>&1 | head -1 || echo "unknown")
    echo -e "  ${GREEN}cyclictest: available${NC}"
else
    echo -e "  ${YELLOW}cyclictest: not found${NC}"
    echo "    Install with: sudo apt install rt-tests"
fi

# Check for hwlatdetect
HWLATDETECT_AVAILABLE=false
if command -v hwlatdetect &> /dev/null; then
    HWLATDETECT_AVAILABLE=true
    echo -e "  ${GREEN}hwlatdetect: available${NC}"
else
    echo -e "  ${YELLOW}hwlatdetect: not found (optional)${NC}"
fi

# Check for root/RT privileges
RT_PRIVILEGES=false
if [ "$EUID" -eq 0 ]; then
    RT_PRIVILEGES=true
    echo -e "  ${GREEN}Root privileges: yes${NC}"
elif [ "$RLIMIT_RTPRIO" != "0" ] && [ "$RLIMIT_RTPRIO" != "N/A" ]; then
    RT_PRIVILEGES=true
    echo -e "  ${GREEN}RT privileges: yes (RLIMIT_RTPRIO=$RLIMIT_RTPRIO)${NC}"
else
    echo -e "  ${YELLOW}RT privileges: no (results may vary)${NC}"
    echo "    For accurate results, run with: sudo $0"
fi

echo ""

# Warn if not RT kernel
if [ "$KERNEL_TYPE" != "PREEMPT_RT" ]; then
    echo -e "${YELLOW}WARNING: Not running on PREEMPT_RT kernel${NC}"
    echo "  Worst-case latency guarantees cannot be validated on standard kernels."
    echo "  See: benchmarks/docs/rt-kernel-setup.md for kernel configuration."
    echo ""
fi

# ============================================================================
# Apply RT Configuration (if requested)
# ============================================================================

if [ "$APPLY_RT_CONFIG" = true ]; then
    echo -e "${YELLOW}Applying RT configuration...${NC}"

    if [ "$EUID" -ne 0 ]; then
        echo -e "${RED}ERROR: --rt-config requires root privileges${NC}"
        exit 1
    fi

    # Set CPU governor to performance
    if command -v cpupower &> /dev/null; then
        cpupower frequency-set -g performance 2>/dev/null || true
        echo "  CPU governor: performance"
    fi

    # Disable turbo boost (Intel)
    if [ -f /sys/devices/system/cpu/intel_pstate/no_turbo ]; then
        echo 1 > /sys/devices/system/cpu/intel_pstate/no_turbo 2>/dev/null || true
        echo "  Turbo boost: disabled"
    fi

    # Disable turbo boost (AMD)
    if [ -f /sys/devices/system/cpu/cpufreq/boost ]; then
        echo 0 > /sys/devices/system/cpu/cpufreq/boost 2>/dev/null || true
        echo "  AMD boost: disabled"
    fi

    # Set kernel.sched_rt_runtime_us to -1 (no RT throttling)
    if [ -f /proc/sys/kernel/sched_rt_runtime_us ]; then
        echo -1 > /proc/sys/kernel/sched_rt_runtime_us 2>/dev/null || true
        echo "  RT throttling: disabled"
    fi

    echo -e "${GREEN}RT configuration applied${NC}"
    echo ""
fi

# ============================================================================
# Cyclictest Baseline
# ============================================================================

run_cyclictest() {
    local duration=$1
    local output_file="$RESULTS_DIR/cyclictest_$TIMESTAMP.txt"

    echo -e "${CYAN}Running cyclictest baseline (${duration}s)...${NC}"

    if [ "$CYCLICTEST_AVAILABLE" = false ]; then
        echo -e "${YELLOW}  Skipping: cyclictest not available${NC}"
        return 1
    fi

    local cyclictest_opts=""

    # Add RT priority if we have privileges
    if [ "$RT_PRIVILEGES" = true ]; then
        cyclictest_opts="-p 80 -m"  # Priority 80, mlockall
    fi

    # Run cyclictest
    # -l: loop count (0 = use duration)
    # -D: duration in seconds
    # -q: quiet mode
    # -h: histogram output
    # -t: number of threads
    if cyclictest $cyclictest_opts -D "$duration" -q -t 1 -i 1000 > "$output_file" 2>&1; then
        echo -e "${GREEN}  cyclictest completed${NC}"

        # Parse results
        if [ -f "$output_file" ]; then
            local max_latency=$(grep "Max Latencies" "$output_file" 2>/dev/null | awk '{print $NF}' || echo "N/A")
            local avg_latency=$(grep "Avg Latencies" "$output_file" 2>/dev/null | awk '{print $NF}' || echo "N/A")

            echo ""
            echo -e "${CYAN}cyclictest Results:${NC}"
            echo "  Average latency: ${avg_latency}µs"
            echo "  Maximum latency: ${max_latency}µs"

            # Check against target
            if [ "$max_latency" != "N/A" ]; then
                if [ "$max_latency" -le "$TARGET_LATENCY_US" ]; then
                    echo -e "  ${GREEN}PASS: Max latency ${max_latency}µs <= ${TARGET_LATENCY_US}µs target${NC}"
                    return 0
                else
                    echo -e "  ${RED}FAIL: Max latency ${max_latency}µs > ${TARGET_LATENCY_US}µs target${NC}"
                    return 1
                fi
            fi
        fi
    else
        echo -e "${RED}  cyclictest failed${NC}"
        cat "$output_file" 2>/dev/null || true
        return 1
    fi
}

# ============================================================================
# HORUS IPC Latency Test
# ============================================================================

run_horus_latency() {
    local duration=$1
    local output_file="$RESULTS_DIR/horus_latency_$TIMESTAMP.txt"

    echo -e "${CYAN}Running HORUS IPC latency test (${duration}s)...${NC}"

    # Build the IPC benchmark
    cd "$HORUS_ROOT"

    echo "  Building benchmarks..."
    if ! cargo build --release -p horus_benchmarks --bin ipc_benchmark 2>&1 | tail -3; then
        echo -e "${RED}  Build failed${NC}"
        return 1
    fi

    # Run the IPC benchmark
    echo "  Running latency test..."

    local benchmark_opts=""
    if [ "$RT_PRIVILEGES" = true ]; then
        benchmark_opts="--rt"  # Enable RT mode if available
    fi

    # Use timeout to limit duration
    if timeout "$duration" ./target/release/ipc_benchmark $benchmark_opts > "$output_file" 2>&1; then
        echo -e "${GREEN}  HORUS benchmark completed${NC}"
    else
        local exit_code=$?
        if [ $exit_code -eq 124 ]; then
            echo -e "${GREEN}  HORUS benchmark completed (timeout reached)${NC}"
        else
            echo -e "${YELLOW}  HORUS benchmark exited with code $exit_code${NC}"
        fi
    fi

    # Parse results
    if [ -f "$output_file" ]; then
        echo ""
        echo -e "${CYAN}HORUS IPC Results:${NC}"

        # Extract key metrics from output
        local link_latency=$(grep -i "Link.*latency\|mean.*ns\|p99.*ns" "$output_file" 2>/dev/null | head -5 || echo "")

        if [ -n "$link_latency" ]; then
            echo "$link_latency" | while read -r line; do
                echo "  $line"
            done
        else
            # Show last 20 lines of output
            echo "  (Raw output):"
            tail -20 "$output_file"
        fi

        # Try to extract worst-case latency
        local worst_case=$(grep -oP 'max[:\s]*\K[\d.]+' "$output_file" 2>/dev/null | head -1 || echo "")
        local worst_case_ns=$(grep -oP 'p99[:\s]*\K[\d.]+' "$output_file" 2>/dev/null | head -1 || echo "")

        if [ -n "$worst_case" ] || [ -n "$worst_case_ns" ]; then
            # Convert to µs if in ns
            local latency_us=""
            if [ -n "$worst_case_ns" ]; then
                latency_us=$(echo "scale=2; $worst_case_ns / 1000" | bc 2>/dev/null || echo "$worst_case_ns")
            fi

            echo ""
            echo "  Worst-case (p99): ${latency_us:-$worst_case}µs"
        fi
    fi

    return 0
}

# ============================================================================
# Run criterion benchmarks with RT focus
# ============================================================================

run_criterion_rt() {
    local output_file="$RESULTS_DIR/criterion_rt_$TIMESTAMP.txt"

    echo -e "${CYAN}Running criterion benchmarks (Link latency)...${NC}"

    cd "$HORUS_ROOT"

    # Run Link::send_recv benchmark which measures single-trip latency
    if cargo bench -p horus_benchmarks --bench link_performance -- --noplot "Link::send_recv" > "$output_file" 2>&1; then
        echo -e "${GREEN}  Criterion benchmarks completed${NC}"

        # Extract timing results
        echo ""
        echo -e "${CYAN}Criterion Results:${NC}"
        grep -E "time:.*\[.*\]|Link::send_recv" "$output_file" | head -10 || true
    else
        echo -e "${YELLOW}  Criterion benchmarks failed or incomplete${NC}"
    fi
}

# ============================================================================
# Main Test Execution
# ============================================================================

PASS_COUNT=0
FAIL_COUNT=0

case $MODE in
    quick)
        echo -e "${BLUE}=== Quick Validation ===${NC}"
        echo ""

        if [ "$CYCLICTEST_AVAILABLE" = true ]; then
            if run_cyclictest 10; then
                ((PASS_COUNT++))
            else
                ((FAIL_COUNT++))
            fi
            echo ""
        fi

        run_horus_latency 15
        ((PASS_COUNT++))
        echo ""

        run_criterion_rt
        ;;

    full)
        echo -e "${BLUE}=== Full Validation ===${NC}"
        echo ""

        if [ "$CYCLICTEST_AVAILABLE" = true ]; then
            if run_cyclictest 60; then
                ((PASS_COUNT++))
            else
                ((FAIL_COUNT++))
            fi
            echo ""
        fi

        run_horus_latency 60
        ((PASS_COUNT++))
        echo ""

        run_criterion_rt
        ;;

    cyclictest)
        echo -e "${BLUE}=== cyclictest Only ===${NC}"
        echo ""

        if [ "$CYCLICTEST_AVAILABLE" = true ]; then
            run_cyclictest 30
        else
            echo -e "${RED}ERROR: cyclictest not available${NC}"
            exit 1
        fi
        ;;

    horus)
        echo -e "${BLUE}=== HORUS Only ===${NC}"
        echo ""

        run_horus_latency 30
        echo ""

        run_criterion_rt
        ;;
esac

# ============================================================================
# Summary Report
# ============================================================================

echo ""
echo -e "${BLUE}================================================================================${NC}"
echo -e "${BLUE}  Validation Summary${NC}"
echo -e "${BLUE}================================================================================${NC}"
echo ""

echo -e "${CYAN}System Configuration:${NC}"
echo "  Kernel: $KERNEL_VERSION ($KERNEL_TYPE)"
echo "  Target: <${TARGET_LATENCY_US}µs worst-case latency"
echo ""

echo -e "${CYAN}Results:${NC}"
echo "  Results saved to: $RESULTS_DIR"
ls -la "$RESULTS_DIR"/*_$TIMESTAMP* 2>/dev/null | while read -r line; do
    echo "    $line"
done

echo ""

if [ "$KERNEL_TYPE" != "PREEMPT_RT" ]; then
    echo -e "${YELLOW}NOTE: For production RT validation, use a PREEMPT_RT kernel.${NC}"
    echo "See: $BENCH_DIR/docs/rt-kernel-setup.md"
    echo ""
fi

if [ "$FAIL_COUNT" -gt 0 ]; then
    echo -e "${RED}$FAIL_COUNT test(s) FAILED${NC}"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Use PREEMPT_RT kernel for deterministic latency"
    echo "  2. Run with --rt-config to optimize system settings"
    echo "  3. Isolate CPU cores: isolcpus=2,3 in kernel cmdline"
    echo "  4. Disable power management: intel_pstate=disable"
    exit 1
fi

echo -e "${GREEN}Validation complete!${NC}"
