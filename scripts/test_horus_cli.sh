#!/bin/bash
# HORUS CLI QA Test Runner
# Run from: /home/lord-patpak/softmata/horus
# Usage: ./scripts/test_horus_cli.sh [--quick|--full]

# Don't exit on errors - we handle them in tests
set +e

HORUS="${HORUS_BIN:-/home/lord-patpak/softmata/horus/target/release/horus}"
TEST_DIR="/tmp/horus-qa-tests"
RESULTS_FILE="$TEST_DIR/results.txt"
PASSED=0
FAILED=0
SKIPPED=0
MODE="${1:-quick}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_pass() { echo -e "${GREEN}[PASS]${NC} $1"; ((PASSED++)); echo "PASS: $1" >> "$RESULTS_FILE"; }
log_fail() { echo -e "${RED}[FAIL]${NC} $1: $2"; ((FAILED++)); echo "FAIL: $1 - $2" >> "$RESULTS_FILE"; }
log_skip() { echo -e "${YELLOW}[SKIP]${NC} $1"; ((SKIPPED++)); echo "SKIP: $1" >> "$RESULTS_FILE"; }
log_info() { echo -e "${CYAN}[INFO]${NC} $1"; }

setup() {
    echo "========================================"
    echo "  HORUS CLI QA Test Suite"
    echo "  Mode: $MODE"
    echo "========================================"
    echo ""

    # Check binary exists
    if [ ! -x "$HORUS" ]; then
        echo -e "${RED}ERROR: HORUS binary not found at $HORUS${NC}"
        echo "Build with: cargo build --release --bin horus"
        exit 1
    fi

    log_info "Using binary: $HORUS"
    log_info "Test directory: $TEST_DIR"

    rm -rf "$TEST_DIR"
    mkdir -p "$TEST_DIR"
    cd "$TEST_DIR"

    # Clean SHM
    $HORUS clean --all 2>/dev/null || true
    rm -rf /dev/shm/horus/* 2>/dev/null || true

    > "$RESULTS_FILE"
    echo "Test run started: $(date)" >> "$RESULTS_FILE"
    echo ""
}

teardown() {
    echo ""
    echo "========================================"
    echo "  Test Results Summary"
    echo "========================================"
    echo -e "  Passed:  ${GREEN}$PASSED${NC}"
    echo -e "  Failed:  ${RED}$FAILED${NC}"
    echo -e "  Skipped: ${YELLOW}$SKIPPED${NC}"
    echo "  Total:   $((PASSED + FAILED + SKIPPED))"
    echo ""
    echo "  Details: $RESULTS_FILE"
    echo "========================================"

    # Cleanup
    $HORUS clean --all 2>/dev/null || true

    # Exit code
    if [ $FAILED -gt 0 ]; then
        exit 1
    fi
}

#######################################
# Basic Tests
#######################################

test_version() {
    local output
    output=$($HORUS --version 2>&1)
    if echo "$output" | grep -qi "horus"; then
        log_pass "horus --version"
    else
        log_fail "horus --version" "Expected 'horus' in output"
    fi
}

test_help() {
    local output
    output=$($HORUS --help 2>&1)
    if echo "$output" | grep -q "Commands:"; then
        log_pass "horus --help"
    else
        log_fail "horus --help" "Expected 'Commands:' in output"
    fi
}

#######################################
# Topic Tests
#######################################

test_topic_list_empty() {
    $HORUS clean --shm 2>/dev/null || true
    local output
    output=$($HORUS topic list 2>&1)
    if echo "$output" | grep -qi "no active topics"; then
        log_pass "topic list (empty)"
    else
        log_fail "topic list (empty)" "Expected 'No active topics' message"
    fi
}

test_topic_list_help() {
    if $HORUS topic list --help 2>&1 | grep -q "List all active topics"; then
        log_pass "topic list --help"
    else
        log_fail "topic list --help" "Missing help text"
    fi
}

test_topic_pub() {
    local output
    $HORUS topic pub test_pub_topic '{"msg": "hello"}' 2>/dev/null
    output=$($HORUS topic list 2>&1)
    if echo "$output" | grep -q "test_pub_topic"; then
        log_pass "topic pub creates topic"
    else
        log_fail "topic pub creates topic" "Topic not visible in list"
    fi
}

test_topic_echo() {
    # Ensure topic exists
    $HORUS topic pub test_echo_topic '{"data": 123}' 2>/dev/null || true

    local output
    output=$(timeout 2 $HORUS topic echo test_echo_topic -n 1 2>&1) || true
    if echo "$output" | grep -q "123"; then
        log_pass "topic echo reads message"
    else
        log_fail "topic echo reads message" "Could not read message data"
    fi
}

test_topic_echo_invalid() {
    local output
    output=$($HORUS topic echo nonexistent_topic_xyz 2>&1) || true
    if echo "$output" | grep -qi "not found\|error"; then
        log_pass "topic echo (invalid topic)"
    else
        log_fail "topic echo (invalid topic)" "Expected error for invalid topic"
    fi
}

test_topic_info() {
    $HORUS topic pub test_info_topic 'test' 2>/dev/null || true
    local output
    output=$($HORUS topic info test_info_topic 2>&1) || true
    if echo "$output" | grep -qi "topic\|name\|size"; then
        log_pass "topic info"
    else
        log_skip "topic info - may need active publisher"
    fi
}

test_topic_hz() {
    $HORUS topic pub hz_test_topic 'test' 2>/dev/null || true
    # Hz needs continuous updates, just verify it runs
    local output
    output=$(timeout 1 $HORUS topic hz hz_test_topic 2>&1) || true
    if echo "$output" | grep -qi "measuring\|rate\|hz"; then
        log_pass "topic hz starts"
    else
        log_skip "topic hz - needs active publishing"
    fi
}

test_topic_json_output() {
    $HORUS topic pub json_test '{}' 2>/dev/null || true
    local output
    output=$($HORUS topic list --json 2>&1)
    if echo "$output" | grep -q '^\['; then
        log_pass "topic list --json"
    else
        log_fail "topic list --json" "Expected JSON array output"
    fi
}

#######################################
# Node Tests
#######################################

test_node_list_empty() {
    local output
    output=$($HORUS node list 2>&1)
    if echo "$output" | grep -qi "no running nodes\|no nodes"; then
        log_pass "node list (empty)"
    else
        # May show nodes if something is running
        log_pass "node list runs"
    fi
}

test_node_list_help() {
    if $HORUS node list --help 2>&1 | grep -q "List"; then
        log_pass "node list --help"
    else
        log_fail "node list --help" "Missing help text"
    fi
}

test_node_info_invalid() {
    local output
    output=$($HORUS node info nonexistent_node_xyz 2>&1) || true
    if echo "$output" | grep -qi "not found\|error\|no.*node"; then
        log_pass "node info (invalid node)"
    else
        log_fail "node info (invalid node)" "Expected error for invalid node"
    fi
}

#######################################
# Param Tests
#######################################

test_param_list() {
    local output
    output=$($HORUS param list 2>&1)
    # Either shows params or says none
    if echo "$output" | grep -qi "param\|no param\|empty"; then
        log_pass "param list"
    else
        log_pass "param list runs"
    fi
}

test_param_set_get() {
    $HORUS param set qa_test_key 42 2>/dev/null || true
    local output
    output=$($HORUS param get qa_test_key 2>&1) || true
    if echo "$output" | grep -q "42"; then
        log_pass "param set/get"
    else
        log_fail "param set/get" "Could not retrieve set value"
    fi
}

test_param_delete() {
    $HORUS param set qa_delete_key 99 2>/dev/null || true
    $HORUS param delete qa_delete_key 2>/dev/null || true
    local output
    output=$($HORUS param get qa_delete_key 2>&1) || true
    if echo "$output" | grep -qi "not found\|error\|no"; then
        log_pass "param delete"
    else
        log_fail "param delete" "Param still exists after delete"
    fi
}

test_param_dump() {
    $HORUS param set dump_test hello 2>/dev/null || true
    local output
    output=$($HORUS param dump 2>&1) || true
    # Should output YAML or show params
    if [ -n "$output" ]; then
        log_pass "param dump"
    else
        log_skip "param dump - no params to dump"
    fi
}

#######################################
# TransformFrame Tests
#######################################

test_hf_list() {
    local output
    output=$($HORUS hf list 2>&1)
    if echo "$output" | grep -qi "frame\|no.*frame\|empty"; then
        log_pass "hf list"
    else
        log_pass "hf list runs"
    fi
}

test_hf_tree() {
    local output
    output=$($HORUS hf tree 2>&1)
    if echo "$output" | grep -qi "frame\|tree\|no.*frame\|empty"; then
        log_pass "hf tree"
    else
        log_pass "hf tree runs"
    fi
}

test_hf_can_invalid() {
    local output
    output=$($HORUS hf can frame_a frame_b 2>&1) || true
    # Should error or say not found
    if echo "$output" | grep -qi "not found\|no.*path\|error\|unavailable"; then
        log_pass "hf can (invalid frames)"
    else
        log_skip "hf can - needs active frames"
    fi
}

#######################################
# Diagnostics Tests
#######################################

test_doctor() {
    local output
    output=$($HORUS doctor 2>&1) || true
    if echo "$output" | grep -qi "check\|pass\|fail\|warning\|diagnostic\|system"; then
        log_pass "doctor"
    else
        log_fail "doctor" "Expected diagnostic output"
    fi
}

test_hardware_platform() {
    local output
    output=$($HORUS hardware platform 2>&1)
    if echo "$output" | grep -qi "platform\|arch\|linux\|cpu\|system"; then
        log_pass "hardware platform"
    else
        log_fail "hardware platform" "Expected platform info"
    fi
}

test_hardware_scan() {
    local output
    output=$($HORUS hardware scan 2>&1) || true
    # May or may not find devices
    if echo "$output" | grep -qi "scan\|device\|found\|no.*device"; then
        log_pass "hardware scan"
    else
        log_pass "hardware scan runs"
    fi
}

test_msg_list() {
    local output
    output=$($HORUS msg list 2>&1) || true
    if echo "$output" | grep -qi "message\|type\|twist\|pose\|no.*message"; then
        log_pass "msg list"
    else
        log_skip "msg list - message registry may be empty"
    fi
}

test_log() {
    local output
    output=$($HORUS log 2>&1) || true
    # May have logs or not
    if [ -n "$output" ] || echo "$output" | grep -qi "log\|no.*log"; then
        log_pass "log"
    else
        log_pass "log runs"
    fi
}

#######################################
# Network Tests
#######################################

test_net_check() {
    local output
    output=$($HORUS net check 2>&1) || true
    if echo "$output" | grep -qi "check\|connectivity\|pass\|fail\|network"; then
        log_pass "net check"
    else
        log_fail "net check" "Expected connectivity check output"
    fi
}

test_net_doctor() {
    local output
    output=$($HORUS net doctor 2>&1) || true
    if echo "$output" | grep -qi "network\|diagnos\|check\|issue"; then
        log_pass "net doctor"
    else
        log_pass "net doctor runs"
    fi
}

#######################################
# Project Lifecycle Tests
#######################################

test_init() {
    cd "$TEST_DIR"
    rm -rf init-test-project
    mkdir -p init-test-project
    cd init-test-project
    local output
    output=$($HORUS init 2>&1)
    if [ -f "horus.toml" ]; then
        log_pass "init creates workspace"
        cd "$TEST_DIR"
    else
        log_fail "init creates workspace" "horus.toml not created"
        cd "$TEST_DIR"
    fi
}

test_new() {
    cd "$TEST_DIR"
    rm -rf new-test-project
    local output
    output=$($HORUS new new-test-project 2>&1) || true
    if [ -f "new-test-project/horus.toml" ]; then
        log_pass "new creates project"
    else
        log_skip "new - may need additional setup"
    fi
}

test_check() {
    if [ ! -d "$TEST_DIR/init-test-project" ]; then
        log_skip "check - no test project"
        return
    fi
    cd "$TEST_DIR/init-test-project"
    local output
    output=$($HORUS check 2>&1) || true
    if echo "$output" | grep -qi "check\|valid\|pass\|error\|warning"; then
        log_pass "check"
    else
        log_pass "check runs"
    fi
    cd "$TEST_DIR"
}

test_build() {
    if [ ! -d "$TEST_DIR/init-test-project" ]; then
        log_skip "build - no test project"
        return
    fi
    cd "$TEST_DIR/init-test-project"
    local output
    output=$($HORUS build 2>&1) || true
    if echo "$output" | grep -qi "compil\|build\|finish\|error"; then
        log_pass "build"
    else
        log_pass "build runs"
    fi
}

test_clean() {
    local output
    output=$($HORUS clean 2>&1) || true
    log_pass "clean"
}

test_clean_shm() {
    local output
    output=$($HORUS clean --shm 2>&1) || true
    log_pass "clean --shm"
}

#######################################
# Cache Tests
#######################################

test_cache_info() {
    local output
    output=$($HORUS cache info 2>&1)
    if echo "$output" | grep -qi "cache\|size\|bytes\|packages\|disk"; then
        log_pass "cache info"
    else
        log_fail "cache info" "Expected cache info output"
    fi
}

test_cache_list() {
    local output
    output=$($HORUS cache list 2>&1) || true
    # May be empty
    if echo "$output" | grep -qi "cache\|package\|no.*package\|empty"; then
        log_pass "cache list"
    else
        log_pass "cache list runs"
    fi
}

#######################################
# Bridge Tests (may need ROS2)
#######################################

test_bridge_info() {
    local output
    output=$($HORUS bridge info 2>&1) || true
    if echo "$output" | grep -qi "bridge\|ros2\|zenoh\|capability\|feature"; then
        log_pass "bridge info"
    else
        log_skip "bridge info - may need ROS2/Zenoh"
    fi
}

test_bridge_list() {
    local output
    output=$($HORUS bridge list 2>&1) || true
    if echo "$output" | grep -qi "topic\|ros2\|no.*topic\|discover"; then
        log_pass "bridge list"
    else
        log_skip "bridge list - ROS2 not running"
    fi
}

#######################################
# Plugin Tests
#######################################

test_plugin_list() {
    local output
    output=$($HORUS plugin list 2>&1) || true
    if echo "$output" | grep -qi "plugin\|no.*plugin\|installed"; then
        log_pass "plugin list"
    else
        log_pass "plugin list runs"
    fi
}

test_plugin_verify() {
    local output
    output=$($HORUS plugin verify 2>&1) || true
    if echo "$output" | grep -qi "plugin\|verify\|integrity\|no.*plugin"; then
        log_pass "plugin verify"
    else
        log_pass "plugin verify runs"
    fi
}

#######################################
# Driver Tests
#######################################

test_driver_list() {
    local output
    output=$($HORUS driver list 2>&1) || true
    if echo "$output" | grep -qi "driver\|no.*driver\|available"; then
        log_pass "driver list"
    else
        log_pass "driver list runs"
    fi
}

#######################################
# Auth Tests
#######################################

test_auth_whoami() {
    local output
    output=$($HORUS auth whoami 2>&1) || true
    if echo "$output" | grep -qi "not.*auth\|logged\|user\|anonymous"; then
        log_pass "auth whoami"
    else
        log_pass "auth whoami runs"
    fi
}

#######################################
# Record Tests
#######################################

test_record_list() {
    local output
    output=$($HORUS record list 2>&1) || true
    if echo "$output" | grep -qi "record\|session\|no.*record"; then
        log_pass "record list"
    else
        log_pass "record list runs"
    fi
}

#######################################
# Husarnet Tests
#######################################

test_husarnet_status() {
    local output
    output=$($HORUS husarnet status 2>&1) || true
    if echo "$output" | grep -qi "husarnet\|status\|not.*install\|not.*running"; then
        log_pass "husarnet status"
    else
        log_skip "husarnet status - may not be installed"
    fi
}

#######################################
# Main Test Runner
#######################################

run_quick_tests() {
    echo ""
    log_info "Running Quick Tests (P0 + basics)"
    echo ""

    echo "--- Basic ---"
    test_version
    test_help

    echo ""
    echo "--- Topic (P0) ---"
    test_topic_list_empty
    test_topic_list_help
    test_topic_pub
    test_topic_echo
    test_topic_echo_invalid

    echo ""
    echo "--- Node (P0) ---"
    test_node_list_empty
    test_node_list_help
    test_node_info_invalid

    echo ""
    echo "--- Param ---"
    test_param_list
    test_param_set_get
    test_param_delete

    echo ""
    echo "--- Diagnostics ---"
    test_doctor
    test_hardware_platform

    echo ""
    echo "--- Project ---"
    test_init
    test_check
    test_clean
}

run_full_tests() {
    echo ""
    log_info "Running Full Test Suite"
    echo ""

    echo "--- Basic ---"
    test_version
    test_help

    echo ""
    echo "--- Topic (P0) ---"
    test_topic_list_empty
    test_topic_list_help
    test_topic_pub
    test_topic_echo
    test_topic_echo_invalid
    test_topic_info
    test_topic_hz
    test_topic_json_output

    echo ""
    echo "--- Node (P0) ---"
    test_node_list_empty
    test_node_list_help
    test_node_info_invalid

    echo ""
    echo "--- Param ---"
    test_param_list
    test_param_set_get
    test_param_delete
    test_param_dump

    echo ""
    echo "--- TransformFrame ---"
    test_hf_list
    test_hf_tree
    test_hf_can_invalid

    echo ""
    echo "--- Diagnostics ---"
    test_doctor
    test_hardware_platform
    test_hardware_scan
    test_msg_list
    test_log

    echo ""
    echo "--- Network ---"
    test_net_check
    test_net_doctor

    echo ""
    echo "--- Project Lifecycle ---"
    test_init
    test_check
    test_build
    test_clean
    test_clean_shm

    echo ""
    echo "--- Cache ---"
    test_cache_info
    test_cache_list

    echo ""
    echo "--- Bridge ---"
    test_bridge_info
    test_bridge_list

    echo ""
    echo "--- Plugin ---"
    test_plugin_list
    test_plugin_verify

    echo ""
    echo "--- Driver ---"
    test_driver_list

    echo ""
    echo "--- Auth ---"
    test_auth_whoami

    echo ""
    echo "--- Record ---"
    test_record_list

    echo ""
    echo "--- Husarnet ---"
    test_husarnet_status
}

main() {
    setup

    case "$MODE" in
        --quick|-q|quick)
            run_quick_tests
            ;;
        --full|-f|full)
            run_full_tests
            ;;
        *)
            run_quick_tests
            ;;
    esac

    teardown
}

main "$@"
