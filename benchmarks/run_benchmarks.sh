#!/bin/bash
# HORUS Benchmark Runner
#
# Usage:
#   ./run_benchmarks.sh           # Run all benchmarks
#   ./run_benchmarks.sh --quick   # Quick run (main benchmark only)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}"
echo "╔════════════════════════════════════════════════════════════╗"
echo "║                   HORUS Benchmarks                         ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Check CPU governor
CPU_GOVERNOR=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo "unknown")
if [ "$CPU_GOVERNOR" != "performance" ]; then
    echo -e "${YELLOW}⚠ CPU governor is '$CPU_GOVERNOR', not 'performance'${NC}"
    echo -e "${YELLOW}  For accurate results: sudo cpupower frequency-set -g performance${NC}"
    echo ""
fi

# Build
echo -e "${GREEN}Building benchmarks...${NC}"
cargo build --release -p horus_benchmarks

# Quick mode - just run main benchmark
if [ "$1" = "--quick" ]; then
    echo ""
    echo -e "${GREEN}Running main benchmark (all_paths_latency)...${NC}"
    cargo run --release -p horus_benchmarks --bin all_paths_latency
    exit 0
fi

# Full run
echo ""
echo -e "${GREEN}▶ all_paths_latency${NC}"
cargo run --release -p horus_benchmarks --bin all_paths_latency

echo ""
echo -e "${GREEN}▶ cross_process_benchmark${NC}"
cargo run --release -p horus_benchmarks --bin cross_process_benchmark

echo ""
echo -e "${GREEN}▶ robotics_messages_benchmark${NC}"
cargo run --release -p horus_benchmarks --bin robotics_messages_benchmark

echo ""
echo -e "${GREEN}▶ determinism_benchmark${NC}"
cargo run --release -p horus_benchmarks --bin determinism_benchmark

echo ""
echo -e "${GREEN}✓ All benchmarks complete${NC}"
