#!/bin/bash
# Run horus user workflow tests across all supported Linux distros via Docker.
#
# Usage:
#   ./tests/docker/run_all.sh              # Test all distros
#   ./tests/docker/run_all.sh ubuntu       # Test one distro
#   ./tests/docker/run_all.sh --quick      # Build only (no full workflow)
#
# Requires: docker

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HORUS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

DISTROS=("ubuntu" "debian" "fedora" "alpine" "arch")
FAILURES=0
SUCCESSES=0

# Parse args
TARGET="${1:-all}"
QUICK=false
[ "$1" = "--quick" ] && QUICK=true && TARGET="all"

if [ "$TARGET" != "all" ] && [ "$TARGET" != "--quick" ]; then
    DISTROS=("$TARGET")
fi

echo -e "${CYAN}════════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}  HORUS Multi-Distro Docker Test Suite${NC}"
echo -e "${CYAN}  Testing: ${DISTROS[*]}${NC}"
echo -e "${CYAN}════════════════════════════════════════════════════════${NC}"
echo ""

for distro in "${DISTROS[@]}"; do
    DOCKERFILE="$SCRIPT_DIR/Dockerfile.${distro}"
    IMAGE_NAME="horus-test-${distro}"

    if [ ! -f "$DOCKERFILE" ]; then
        echo -e "${YELLOW}[SKIP] No Dockerfile for ${distro}${NC}"
        continue
    fi

    echo -e "${CYAN}── Building ${distro} ──${NC}"

    # Build the Docker image
    if docker build \
        -f "$DOCKERFILE" \
        -t "$IMAGE_NAME" \
        "$HORUS_ROOT" 2>&1 | tail -5; then
        echo -e "${GREEN}[OK] ${distro} image built${NC}"
    else
        echo -e "${RED}[FAIL] ${distro} build failed${NC}"
        FAILURES=$((FAILURES + 1))
        continue
    fi

    if [ "$QUICK" = true ]; then
        echo -e "${GREEN}[OK] ${distro}: build-only mode (--quick)${NC}"
        SUCCESSES=$((SUCCESSES + 1))
        continue
    fi

    echo -e "${CYAN}── Testing ${distro} ──${NC}"

    # Run the full workflow test
    if docker run --rm \
        --tmpfs /dev/shm:rw,nosuid,nodev,exec,size=256m \
        -v "$SCRIPT_DIR/test_user_workflow.sh:/horus/tests/docker/test_user_workflow.sh:ro" \
        "$IMAGE_NAME" \
        bash /horus/tests/docker/test_user_workflow.sh 2>&1; then
        echo -e "${GREEN}[OK] ${distro}: all workflow tests passed${NC}"
        SUCCESSES=$((SUCCESSES + 1))
    else
        echo -e "${RED}[FAIL] ${distro}: workflow tests failed${NC}"
        FAILURES=$((FAILURES + 1))
    fi

    echo ""
done

# Summary
echo -e "${CYAN}════════════════════════════════════════════════════════${NC}"
echo -e "  ${GREEN}Passed: ${SUCCESSES}${NC}  ${RED}Failed: ${FAILURES}${NC}"
echo -e "${CYAN}════════════════════════════════════════════════════════${NC}"

exit $FAILURES
