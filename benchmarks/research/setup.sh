#!/bin/bash
# HORUS Benchmark Environment Setup
#
# Configures the system for reproducible, low-variance measurements.
# Run with: sudo ./setup.sh
# Undo with: sudo ./setup.sh --teardown
#
# Requirements:
#   - Linux x86_64 with constant_tsc CPU flag
#   - Root or sudo access
#   - cpupower (linux-tools-common) for governor control

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

# ── Teardown mode ──────────────────────────────────────────────────────

if [ "$1" = "--teardown" ]; then
    echo -e "${CYAN}Restoring default system settings...${NC}"

    # Restore CPU governor
    if command -v cpupower &>/dev/null; then
        cpupower frequency-set -g schedutil 2>/dev/null || \
        cpupower frequency-set -g ondemand 2>/dev/null || true
        echo -e "  ${GREEN}[+]${NC} CPU Governor: restored to default"
    fi

    # Re-enable turbo boost
    if [ -f /sys/devices/system/cpu/intel_pstate/no_turbo ]; then
        echo 0 > /sys/devices/system/cpu/intel_pstate/no_turbo
        echo -e "  ${GREEN}[+]${NC} Turbo Boost: re-enabled (Intel)"
    elif [ -f /sys/devices/system/cpu/cpufreq/boost ]; then
        echo 1 > /sys/devices/system/cpu/cpufreq/boost
        echo -e "  ${GREEN}[+]${NC} Turbo Boost: re-enabled (AMD)"
    fi

    echo -e "${GREEN}System restored.${NC}"
    exit 0
fi

# ── Environment report ─────────────────────────────────────────────────

echo -e "${CYAN}"
echo "╔════════════════════════════════════════════════════════════╗"
echo "║           HORUS Benchmark Environment Setup                  ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

echo -e "${CYAN}System Information:${NC}"
echo -e "  CPU:      $(grep 'model name' /proc/cpuinfo 2>/dev/null | head -1 | cut -d: -f2 | xargs || echo 'unknown')"
echo -e "  Cores:    $(nproc) logical ($(grep 'physical id' /proc/cpuinfo 2>/dev/null | sort -u | wc -l || echo '?') sockets)"
echo -e "  Kernel:   $(uname -r)"
echo -e "  Arch:     $(uname -m)"
echo -e "  Rust:     $(rustc --version 2>/dev/null || echo 'not found')"
echo -e "  Hostname: $(hostname)"
echo ""

# ── Check prerequisites ───────────────────────────────────────────────

WARNINGS=0

# Check constant_tsc (required for reliable RDTSC timing)
if grep -q constant_tsc /proc/cpuinfo 2>/dev/null; then
    echo -e "  ${GREEN}[+]${NC} constant_tsc: available (RDTSC reliable across cores)"
else
    echo -e "  ${YELLOW}[!]${NC} constant_tsc: NOT available (RDTSC may drift between cores)"
    WARNINGS=$((WARNINGS + 1))
fi

# Check nohz_full / isolcpus
ISOLATED=$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo "none")
if [ "$ISOLATED" != "none" ] && [ -n "$ISOLATED" ]; then
    echo -e "  ${GREEN}[+]${NC} Isolated cores: $ISOLATED"
else
    echo -e "  ${YELLOW}[!]${NC} No isolated cores (add isolcpus=2,3 to kernel boot params for best results)"
    WARNINGS=$((WARNINGS + 1))
fi

echo ""

# ── Set CPU governor to performance ───────────────────────────────────

echo -e "${CYAN}Configuring CPU:${NC}"

if command -v cpupower &>/dev/null; then
    cpupower frequency-set -g performance 2>/dev/null || true
    GOV=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo "unknown")
    if [ "$GOV" = "performance" ]; then
        echo -e "  ${GREEN}[+]${NC} CPU Governor: performance"
    else
        echo -e "  ${YELLOW}[!]${NC} CPU Governor: $GOV (failed to set performance)"
        WARNINGS=$((WARNINGS + 1))
    fi
else
    echo -e "  ${YELLOW}[!]${NC} cpupower not found (install linux-tools-common)"
    WARNINGS=$((WARNINGS + 1))
fi

# ── Disable turbo boost ───────────────────────────────────────────────

if [ -f /sys/devices/system/cpu/intel_pstate/no_turbo ]; then
    echo 1 > /sys/devices/system/cpu/intel_pstate/no_turbo
    echo -e "  ${GREEN}[+]${NC} Turbo Boost: disabled (Intel pstate)"
elif [ -f /sys/devices/system/cpu/cpufreq/boost ]; then
    echo 0 > /sys/devices/system/cpu/cpufreq/boost
    echo -e "  ${GREEN}[+]${NC} Turbo Boost: disabled (AMD boost)"
else
    echo -e "  ${YELLOW}[!]${NC} Turbo Boost: cannot control (no Intel pstate or AMD boost)"
    WARNINGS=$((WARNINGS + 1))
fi

# ── Check /dev/shm ────────────────────────────────────────────────────

SHM_SIZE=$(df -BM /dev/shm 2>/dev/null | awk 'NR==2{print $2}' || echo "unknown")
echo -e "  ${GREEN}[+]${NC} /dev/shm: $SHM_SIZE available"

echo ""

# ── Summary ───────────────────────────────────────────────────────────

if [ $WARNINGS -eq 0 ]; then
    echo -e "${GREEN}Environment ready for benchmarking. 0 warnings.${NC}"
else
    echo -e "${YELLOW}Environment configured with $WARNINGS warning(s).${NC}"
    echo -e "${YELLOW}Results may have higher variance. See warnings above.${NC}"
fi

echo ""
echo -e "${CYAN}Run benchmarks with:${NC}"
echo "  cargo run --release -p horus_benchmarks --bin raw_baselines"
echo "  cargo run --release -p horus_benchmarks --bin all_paths_latency"
echo ""
echo -e "${CYAN}Restore settings with:${NC}"
echo "  sudo ./research/setup.sh --teardown"
