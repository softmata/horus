#!/bin/bash
# HORUS Benchmark Suite — Full Run
#
# Runs all benchmarks and generates analysis plots.
#
# Usage:
#   ./research/run_all.sh              # Full suite (~30 minutes)
#   ./research/run_all.sh --quick      # Quick validation (~3 minutes)
#   sudo ./research/setup.sh           # Run first for best results

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BENCH_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$BENCH_DIR/.."

RESULTS="$BENCH_DIR/research/results"
mkdir -p "$RESULTS"

GREEN='\033[0;32m'
CYAN='\033[0;36m'
NC='\033[0m'

DURATION=10
CSV_FLAG=""

if [ "$1" = "--quick" ]; then
    DURATION=2
    echo -e "${CYAN}Quick mode: ${DURATION}s per test${NC}"
else
    echo -e "${CYAN}Full mode: ${DURATION}s per test${NC}"
fi

echo -e "${CYAN}"
echo "╔════════════════════════════════════════════════════════════╗"
echo "║           HORUS Benchmark Suite                              ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Check CPU governor
CPU_GOV=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo "unknown")
if [ "$CPU_GOV" != "performance" ]; then
    echo -e "⚠ CPU governor is '$CPU_GOV'. Run: sudo ./research/setup.sh"
    echo ""
fi

# Build all benchmarks
echo -e "${GREEN}[1/7] Building benchmarks...${NC}"
cargo build --release -p horus_benchmarks 2>&1 | grep -E "Compiling horus_benchmarks|Finished" || true

# Raw baselines
echo -e "${GREEN}[2/7] Raw hardware baselines...${NC}"
cargo run --release -p horus_benchmarks --bin raw_baselines -- --json "$RESULTS/baselines.json"
echo ""

# Research latency (sustained + size sweep)
echo -e "${GREEN}[3/7] Latency (sustained, size sweep)...${NC}"
cargo run --release -p horus_benchmarks --bin research_latency -- --duration "$DURATION" --csv "$RESULTS/latency.csv" --json "$RESULTS/latency.json"
echo ""

# Throughput
echo -e "${GREEN}[4/7] Sustained throughput...${NC}"
cargo run --release -p horus_benchmarks --bin research_throughput -- --duration "$DURATION" --csv "$RESULTS/throughput.csv"
echo ""

# Jitter
echo -e "${GREEN}[5/7] RT jitter + IPC contention...${NC}"
cargo run --release -p horus_benchmarks --bin research_jitter -- --duration "$DURATION" --csv "$RESULTS/jitter.csv"
echo ""

# Scalability
echo -e "${GREEN}[6/7] Node + topic scalability...${NC}"
cargo run --release -p horus_benchmarks --bin research_scalability -- --csv "$RESULTS/scale.csv"
echo ""

# Competitor comparison
echo -e "${GREEN}[7/7] Competitor comparison (HORUS vs UDP)...${NC}"
cargo run --release -p horus_benchmarks --bin competitor_comparison -- --duration "$DURATION" --csv "$RESULTS/comparison.csv"
echo ""

# Analysis (if Python available)
if command -v python3 &>/dev/null; then
    echo -e "${GREEN}Generating plots and tables...${NC}"
    python3 "$BENCH_DIR/research/analyze.py" --input-dir "$RESULTS" --output-dir "$RESULTS" 2>&1 || echo "  (analysis requires: pip install matplotlib numpy pandas)"
    echo ""
fi

echo -e "${GREEN}════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}Results in: $RESULTS/${NC}"
echo ""
ls -lh "$RESULTS/" 2>/dev/null || true
